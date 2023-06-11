"""

  Hexapod Control GUI
 ---------------------

"""
import os
import wx
import copy
from enum import Enum
import numpy as np
from gl_helpers.viewer_canvas import ViewerCanvas
from gait_generator import optimise_walking_gait
from translate_position import joints_to_all_leg_positions, get_leg_base_frames
from leg_kinematics import LegKinematics as Kin
from robot import Robot, RobotConnectionFailed
from dynamic_walking import Velocity2D, DynamicGait
from gamepad import GamePad


class ViewMode(Enum):
  Static = 1
  Trajectory = 2
  Dynamic = 3


class MainMenu(wx.MenuBar):

  ID_OPEN = 1001
  ID_QUIT = 1010

  ID_CONNECT_ROBOT = 3001

  ID_GEN_WALKING = 2001
  ID_GEN_TEST = 2002

  def __init__(self, main_window):
    """
    Creates the customised wx.MenuBar object for the controller GUI
    """
    wx.MenuBar.__init__(self)
    self.main_window = main_window

    self.file_menu = wx.Menu()
    self.file_menu.Append(self.ID_OPEN, "Open Trajectory", kind=wx.ITEM_NORMAL)
    self.file_menu.Append(wx.ID_ANY, "", kind=wx.ITEM_SEPARATOR)
    self.file_menu.Append(self.ID_QUIT, "Quit", kind=wx.ITEM_NORMAL)
    self.Append(self.file_menu, '&File')

    self.gen_menu = wx.Menu()
    self.gen_menu.Append(self.ID_GEN_WALKING, "Generate Walking Gait")
    self.gen_menu.AppendSeparator()
    self.gen_menu.Append(self.ID_GEN_TEST, "Generator Servo Test Trajectory")
    self.Append(self.gen_menu, "&Generators")

    main_window.SetMenuBar(self)

  def bind_menu_handler(self, menu_id, callback):
    """
    Helper function to bind the given callback function to a menu item.
    Will override a callback if one is already bound
    :param menu_id: Integer ID of the menu item
    :param callback: Function, callback to bind
    :return:
    """
    menu_item = self.FindItemById(menu_id)
    if menu_item is None:
      print("Error binding menu event handler. Menu item ID %d not found." % menu_id)
    else:
      self.main_window.Bind(wx.EVT_MENU, callback, menu_item)


class MainWindow(wx.Frame):

  def __init__(self):
    wx.Frame.__init__(self, parent=None, id=wx.ID_ANY, title='Hexapod Controller')
    self.status_bar = self.CreateStatusBar()
    self.status_bar.SetStatusText("  No Trajectory Present")
    self.SetMinSize(wx.Size(500, 400))

    self.menu_bar = MainMenu(self)
    self.menu_bar.bind_menu_handler(self.menu_bar.ID_OPEN, self.on_open_trajectory_menu)
    self.menu_bar.bind_menu_handler(self.menu_bar.ID_QUIT, self.on_quit_menu)
    # self.menu_bar.bind_menu_handler(self.menu_bar.ID_CONNECT_ROBOT, self.on_connect_robot)
    self.menu_bar.bind_menu_handler(self.menu_bar.ID_GEN_WALKING, self.gen_walking_gait)
    self.menu_bar.bind_menu_handler(self.menu_bar.ID_GEN_TEST, self.gen_test_trajectory)

    window_sizer = wx.BoxSizer(wx.HORIZONTAL)
    self.SetSizer(window_sizer)

    self.robot_interface = None
    self.game_pad = GamePad()
    self.kin = Kin()
    self.dynamic_gait = DynamicGait(self.kin)

    self.view_mode = ViewMode.Dynamic

    self.joint_trajectory = None
    self.animation_step = 0
    self.animate_timer = wx.Timer(self)
    self.Bind(wx.EVT_TIMER, self.update_frame, self.animate_timer)

    self.base_frames = get_leg_base_frames()
    self.base_frames["body"] = np.eye(4)
    self.frames = self.base_frames

    self.leg_colors = [[1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 1, 1], [0, 0, 1], [1, 0, 1]]

    self.canvas = ViewerCanvas(self)
    window_sizer.Add(self.canvas, proportion=2, flag=wx.EXPAND)
    window_sizer.Add(wx.StaticLine(self, wx.ID_ANY, style=wx.LI_VERTICAL))

    self.side_bar_sizer = wx.BoxSizer(wx.VERTICAL)
    window_sizer.Add(self.side_bar_sizer)

    self.connection_state_text = wx.StaticText(self, wx.ID_ANY, "Robot Not Connected")
    self.side_bar_sizer.Add(self.connection_state_text, 1, wx.CENTER|wx.TOP, 5)
    self.connect_button = wx.Button(self, wx.ID_ANY, label="Connect Robot")
    self.connect_button.Bind(wx.EVT_BUTTON, self.on_connect_robot)
    self.side_bar_sizer.Add(self.connect_button, 1, wx.CENTER|wx.TOP|wx.BOTTOM, 5)
    self.side_bar_sizer.Add(wx.StaticLine(self, wx.ID_ANY, size=wx.Size(200, -1), style=wx.LI_HORIZONTAL))

    self.static_pose_ratio = wx.RadioButton(self, wx.ID_ANY, "Static Pose")
    self.static_pose_ratio.Enable()
    self.trajectory_ratio = wx.RadioButton(self, wx.ID_ANY, "Trajectory")
    self.dynamic_walk_ratio = wx.RadioButton(self, wx.ID_ANY, "Dynamic Walk")
    self.static_pose_ratio.Bind(wx.EVT_RADIOBUTTON, self.view_mode_ratio_evt)
    self.trajectory_ratio.Bind(wx.EVT_RADIOBUTTON, self.view_mode_ratio_evt)
    self.dynamic_walk_ratio.Bind(wx.EVT_RADIOBUTTON, self.view_mode_ratio_evt)
    self.side_bar_sizer.Add(self.static_pose_ratio, 1, wx.LEFT, 20)
    self.side_bar_sizer.Add(self.trajectory_ratio, 1, wx.LEFT, 20)
    self.side_bar_sizer.Add(self.dynamic_walk_ratio, 1, wx.LEFT, 20)
    self.side_bar_sizer.Add(wx.StaticLine(self, wx.ID_ANY, size=wx.Size(200, -1), style=wx.LI_HORIZONTAL))
    self.set_view_mode(ViewMode.Static)

    icon = wx.Icon()
    icon.CopyFromBitmap(wx.Bitmap(os.path.join("resources", "bug_icon.png"), wx.BITMAP_TYPE_ANY))
    self.SetIcon(icon)

    self.Show()

    self.animate_timer.Start(int(1000 / 30.0))

  def view_mode_ratio_evt(self, _):
    if self.static_pose_ratio.GetValue():
      # print("setting static view")
      self.view_mode = ViewMode.Static
    if self.trajectory_ratio.GetValue():
      # print("setting trajectory view")
      self.view_mode = ViewMode.Trajectory
    if self.dynamic_walk_ratio.GetValue():
      # print("setting dyanmic view")
      self.view_mode = ViewMode.Dynamic

  def set_view_mode(self, new_view_mode):
    self.view_mode = new_view_mode
    if new_view_mode == ViewMode.Static:
      self.static_pose_ratio.SetValue(True)
    if new_view_mode == ViewMode.Trajectory:
      self.trajectory_ratio.SetValue(True)
    if new_view_mode == ViewMode.Dynamic:
      self.dynamic_walk_ratio.SetValue(True)

  def on_open_trajectory_menu(self, _):
    print("Sommat")

  def on_quit_menu(self, _):
    self.Close()

  def on_connect_robot(self, _):
    try:
      self.robot_interface = Robot()
      self.connection_state_text.SetLabel(self.robot_interface.connection_msg)
    except RobotConnectionFailed as e:
      self.connection_state_text.SetLabel(str(e))

  def gen_walking_gait(self, _):
      self.joint_trajectory = optimise_walking_gait()
      self.set_view_mode(ViewMode.Trajectory)

  def gen_test_trajectory(self, _):
      self.joint_trajectory = self.generate_testing_trajectory()
      self.set_view_mode(ViewMode.Trajectory)

  def update_frame(self, _):
    if self.view_mode == ViewMode.Static:
      self.canvas.body_frame_velocity_preview = []
      self.frames = copy.copy(self.base_frames)
    if self.view_mode == ViewMode.Trajectory:
      self.canvas.body_frame_velocity_preview = []
      self.frames = copy.copy(self.base_frames)

      if self.joint_trajectory is not None:
        self.animation_step = (self.animation_step + 1) % len(self.joint_trajectory)
        self.status_bar.SetStatusText("laying trajectory sample %d of %d" % (self.animation_step, len(self.joint_trajectory)))

        labels = ["front_right", "middle_right", "rear_right", "rear_left", "middle_left", "front_left"]
        joint_angles = []
        for leg_idx, label in enumerate(labels):
          leg_angles = self.joint_trajectory[self.animation_step][leg_idx*3:leg_idx*3+3]
          joint_angles += leg_angles.tolist()
          leg_frames = self.kin.forwards_all_frames(leg_angles)
          for leg_label in leg_frames.keys():
            leg_frames[leg_label] = np.matmul(leg_frames[leg_label], self.frames[label])
            self.frames[label + "_" + leg_label] = leg_frames[leg_label]

        if self.robot_interface is not None:
          self.robot_interface.send_joint_angles(joint_angles)

    if self.view_mode == ViewMode.Dynamic:
      self.frames = copy.copy(self.base_frames)
      control_inputs = self.game_pad.get_state()
      linear_scale = 0.05  # max linear velocity (m/s)
      angular_scale = 0.5  # max angular velocity (rads/sec)
      # print(control_inputs)
      robot_velocity = Velocity2D(control_inputs['rightx'] * linear_scale,
                                  -control_inputs['righty'] * linear_scale,
                                  control_inputs['leftx'] * angular_scale)

      body_frame = np.eye(4)
      time_step = 1/15
      step_count = 30
      self.canvas.body_frame_velocity_preview = [body_frame]
      for _ in range(step_count):
        body_frame = robot_velocity.apply(body_frame, time_step)
        self.canvas.body_frame_velocity_preview.append(body_frame)

      self.dynamic_gait.plan_using_body_velocity(robot_velocity)

      self.canvas.lines.clear_lines()
      for leg_idx, this_step_line in enumerate(self.dynamic_gait.toe_trajectories_this_step):
        for idx in range(len(this_step_line)-1):
          start = this_step_line[idx]
          end = this_step_line[idx+1]
          self.canvas.lines.add_line(start, end, self.leg_colors[leg_idx][0], self.leg_colors[leg_idx][1], self.leg_colors[leg_idx][2])
      for leg_idx, this_step_line in enumerate(self.dynamic_gait.toe_trajectories_next_step):
        for idx in range(len(this_step_line) - 1):
          start = this_step_line[idx]
          end = this_step_line[idx + 1]
          self.canvas.lines.add_line(start, end, self.leg_colors[leg_idx][0]/2, self.leg_colors[leg_idx][1]/2,
                                     self.leg_colors[leg_idx][2]/2)

      # Add leg working area boundaries
      for leg_idx in range(6):
        for idx in range(self.dynamic_gait.working_area_boundary_point_count):
          start = self.dynamic_gait.working_area_boundaries[leg_idx][idx, :]
          end = self.dynamic_gait.working_area_boundaries[leg_idx][(idx+1) % self.dynamic_gait.working_area_boundary_point_count, :]
          self.canvas.lines.add_line(start, end, self.leg_colors[leg_idx][0], self.leg_colors[leg_idx][1],
                                     self.leg_colors[leg_idx][2])

      self.canvas.lines.update_geometry()

    self.canvas.Refresh()  # update_robot_pose(positions)

  def generate_testing_trajectory(self):
      traj = []
      still_count = 10

      for servo in range(18):
        leg_joint = servo % 3
        servo_angles = [0] * still_count
        servo_angles.extend(np.linspace(0, self.kin.joint_high_limits[leg_joint], 15))
        servo_angles.extend(np.linspace(self.kin.joint_high_limits[leg_joint], 0, 15))
        servo_angles.extend(np.linspace(0, self.kin.joint_low_limits[leg_joint], 15))
        servo_angles.extend(np.linspace(self.kin.joint_low_limits[leg_joint], 0, 15))

        for angle in servo_angles:
          angles = np.zeros(18,)
          angles[servo] = angle
          traj.append(angles)

      return traj


def main():
  """

  :return: None
  """
  gui_app = wx.App(False)
  locale = wx.Locale(wx.LANGUAGE_ENGLISH)  # Strange WX bug fix!
  MainWindow()
  gui_app.MainLoop()


if __name__ == '__main__':
  main()
