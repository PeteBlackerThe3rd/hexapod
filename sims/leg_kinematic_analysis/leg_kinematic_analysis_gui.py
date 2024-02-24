"""

  Hexapod Control GUI
 ---------------------

"""
import os
import wx
import copy
from enum import Enum
import numpy as np
# from gl_helpers.viewer_canvas import ViewerCanvas
# from gait_generator import optimise_walking_gait
# from translate_position import joints_to_all_leg_positions, get_leg_base_frames
# from leg_kinematics import LegKinematics as Kin
# from robot import Robot, RobotConnectionFailed
# from dynamic_walking import Velocity2D, DynamicGait
# from gamepad import GamePad


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


class WorkingVolumeView(wx.Panel):
  def __init__(self, parent):
    super(WorkingVolumeView, self).__init__(parent)
    self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
    self.Bind(wx.EVT_SIZE, self.on_size)
    self.Bind(wx.EVT_PAINT, self.on_paint)

    self.link1_length = 20
    self.link2_length = 40
    self.link3_length = 70

    self.joint1_min = np.radians(45)
    self.joint1_max = np.radians(135)
    self.joint2_min = np.radians(10)
    self.joint2_max = np.radians(135)

    self.sample_count = 50
    self.joint_angle_samples = None
    self.leg_pose_lines = None
    self.update_kinematics()

  def update_kinematics(self):
    self.joint_angle_samples = self.generate_volume_outline()
    self.leg_pose_lines = self.generate_leg_poses()

  def generate_leg_poses(self):

    lines = []
    angles = [[self.joint1_min, self.joint2_min],
              [self.joint1_max, self.joint2_min],
              [self.joint1_min, self.joint2_max],
              [self.joint1_max, self.joint2_max]]

    for j1_angle, j2_angle in angles:
      j1_pos, j2_pos, toe_pos = self.toe_pos(j1_angle, j2_angle)
      lines.append([j1_pos, j2_pos])
      lines.append([j2_pos, toe_pos])

    return lines

  def generate_volume_outline(self):
    joint_angle_samples = []
    for j1_angle in np.linspace(self.joint1_min, self.joint1_max, self.sample_count):
      _, __, toe_pos = self.toe_pos(j1_angle, self.joint2_min)
      joint_angle_samples.append(toe_pos)
    for j2_angle in np.linspace(self.joint2_min, self.joint2_max, self.sample_count):
      _, __, toe_pos = self.toe_pos(self.joint1_max, j2_angle)
      joint_angle_samples.append(toe_pos)
    for j1_angle in np.linspace(self.joint1_max, self.joint1_min, self.sample_count):
      _, __, toe_pos = self.toe_pos(j1_angle, self.joint2_max)
      joint_angle_samples.append(toe_pos)
    for j2_angle in np.linspace(self.joint2_max, self.joint2_min, self.sample_count):
      _, __, toe_pos = self.toe_pos(self.joint1_min, j2_angle)
      joint_angle_samples.append(toe_pos)
    return joint_angle_samples

  def on_size(self, event):
    event.Skip()
    self.Refresh()

  def toe_pos(self, joint1_angle, joint2_angle):

    joint1_pos = np.array([-self.link1_length, 0])
    joint1_angle -= np.pi / 2
    joint1_rotation = np.array([[np.cos(joint1_angle), -np.sin(joint1_angle)],
                                [np.sin(joint1_angle), np.cos(joint1_angle)]])
    joint2_pos = joint1_pos + np.matmul(np.array([-self.link2_length, 0]), joint1_rotation)

    # print("link1: %s" % str(np.array([-self.link2_length, 0])))
    # print("Rotaton: %s" % str(joint1_rotation))
    # print("result: %s" % str(np.matmul(np.array([-self.link2_length, 0]), joint1_rotation)))

    joint2_cr = joint1_angle + joint2_angle
    joint2_rotation = np.array([[np.cos(joint2_cr), -np.sin(joint2_cr)],
                                [np.sin(joint2_cr), np.cos(joint2_cr)]])
    toe_pos = joint2_pos + np.matmul(np.array([-self.link3_length, 0]), joint2_rotation)

    return joint1_pos, joint2_pos, toe_pos

  def on_paint(self, event):
    w, h = self.GetClientSize()
    area_min = np.array([-200, -100])
    area_max = np.array([100, 100])
    area_size = area_max - area_min
    grid_spacing = 10

    # compute scale and offset to fit the area in the centre of the frame
    frame_aspect = w / float(h)
    area_aspect = area_size[0] / float(area_size[1])
    if area_aspect >= frame_aspect:
      scale = w / float(area_size[0])
      offset = np.array([0, (h - (area_size[1] * scale)) / 2.0]) - (area_min * scale)
    else:
      scale = h / float(area_size[1])
      offset = np.array([(w - (area_size[0] * scale)) / 2.0, 0]) - (area_min * scale)

    dc = wx.AutoBufferedPaintDC(self)
    dc.Clear()

    # draw grid
    dc.SetPen(wx.Pen(wx.Colour(180, 180, 180), 1))
    for x_pos in np.linspace(-500, 500, 100):
      y_pos = (x_pos * scale) + offset[1]
      dc.DrawLine(0, y_pos, w, y_pos)
      x_pos = (x_pos * scale) + offset[0]
      dc.DrawLine(x_pos, 0, x_pos, h)

    tl = ((area_min * scale) + offset).astype(int)
    size_scaled = (area_size * scale).astype(int)
    br = ((area_max * scale) + offset).astype(int)

    # dc.DrawRectangle(tl[0], tl[1], size_scaled[0], size_scaled[1])
    # dc.DrawLine(tl[0], tl[1], br[0], br[1])
    dc.SetPen(wx.Pen(wx.BLACK, 1))
    # dc.DrawCircle(w / 2, h / 2, 100)

    j1 = ((np.array([-self.link1_length, 0]) * scale) + offset).astype(int)
    dc.DrawLine(int(offset[0]), int(offset[1]), j1[0], j1[1])

    # dc.SetPen(wx.Pen(wx.GREEN, 3))
    for i in range(len(self.joint_angle_samples) - 1):
      if 0 <= i < 50:
        dc.SetPen(wx.Pen(wx.GREEN, 3))
      elif 50 <= i < 100:
        dc.SetPen(wx.Pen(wx.RED, 3))
      elif 100 <= i < 150:
        dc.SetPen(wx.Pen(wx.BLUE, 3))
      elif 150 <= i < 200:
        dc.SetPen(wx.Pen(wx.YELLOW, 3))
      p1 = ((self.joint_angle_samples[i] * scale) + offset).astype(int)
      p2 = ((self.joint_angle_samples[i+1] * scale) + offset).astype(int)

      # print("p1 (%s)\np2 (%s)" % (p1, p2))
      dc.DrawLine(p1[0], p1[1], p2[0], p2[1])

    # draw leg poses at extremes of envelope
    dc.SetPen(wx.Pen(wx.BLACK, 1))
    for start, end in self.leg_pose_lines:
      p1 = ((start * scale) + offset).astype(int)
      p2 = ((end * scale) + offset).astype(int)
      dc.DrawLine(p1[0], p1[1], p2[0], p2[1])

    # draw j0 singularity
    singularity_en = wx.Pen(wx.RED, 3)

    dc.SetPen(singularity_en)
    dc.DrawLine(offset[0], 0, offset[0], h)

class MainWindow(wx.Frame):

  def __init__(self):
    wx.Frame.__init__(self, parent=None, id=wx.ID_ANY, title='Hexapod Kinematic Analysis')
    self.status_bar = self.CreateStatusBar()
    self.status_bar.SetStatusText("  No Trajectory Present")
    self.SetMinSize(wx.Size(500, 400))

    # self.menu_bar = MainMenu(self)
    # self.menu_bar.bind_menu_handler(self.menu_bar.ID_OPEN, self.on_open_trajectory_menu)
    # self.menu_bar.bind_menu_handler(self.menu_bar.ID_QUIT, self.on_quit_menu)
    # # self.menu_bar.bind_menu_handler(self.menu_bar.ID_CONNECT_ROBOT, self.on_connect_robot)
    # self.menu_bar.bind_menu_handler(self.menu_bar.ID_GEN_WALKING, self.gen_walking_gait)
    # self.menu_bar.bind_menu_handler(self.menu_bar.ID_GEN_TEST, self.gen_test_trajectory)

    window_sizer = wx.BoxSizer(wx.HORIZONTAL)
    self.SetSizer(window_sizer)

    self.canvas = WorkingVolumeView(self)
    window_sizer.Add(self.canvas, proportion=2, flag=wx.EXPAND)
    window_sizer.Add(wx.StaticLine(self, wx.ID_ANY, style=wx.LI_VERTICAL))

    self.side_bar_sizer = wx.BoxSizer(wx.VERTICAL)
    window_sizer.Add(self.side_bar_sizer)

    self.side_bar_sizer.Add(wx.StaticText(self, wx.ID_ANY, "link 1"), 1, wx.CENTER|wx.TOP, 5)
    self.link1_len_spin = wx.SpinCtrlDouble(self, wx.ID_ANY, initial=self.canvas.link1_length,
                                             min=-180, max=180)
    self.link1_len_spin.Bind(wx.EVT_SPINCTRLDOUBLE, self.update_kinematics)
    self.side_bar_sizer.Add(self.link1_len_spin, 1, wx.CENTER|wx.TOP|wx.BOTTOM, 5)

    self.side_bar_sizer.Add(wx.StaticText(self, wx.ID_ANY, "Joint 1"), 1, wx.CENTER|wx.TOP, 5)
    self.link2_len_spin = wx.SpinCtrlDouble(self, wx.ID_ANY, initial=self.canvas.link2_length,
                                             min=-180, max=180)
    self.link2_len_spin.Bind(wx.EVT_SPINCTRLDOUBLE, self.update_kinematics)
    self.side_bar_sizer.Add(self.link2_len_spin, 1, wx.CENTER|wx.TOP|wx.BOTTOM, 5)

    self.joint1_min_spin = wx.SpinCtrlDouble(self, wx.ID_ANY, initial=np.degrees(self.canvas.joint1_min),
                                             min=-180, max=180)
    self.joint1_min_spin.Bind(wx.EVT_SPINCTRLDOUBLE, self.update_kinematics)
    self.side_bar_sizer.Add(self.joint1_min_spin, 1, wx.CENTER|wx.TOP|wx.BOTTOM, 5)

    self.joint1_max_spin = wx.SpinCtrlDouble(self, wx.ID_ANY, initial=np.degrees(self.canvas.joint1_max),
                                             min=-180, max=180)
    self.joint1_max_spin.Bind(wx.EVT_SPINCTRLDOUBLE, self.update_kinematics)
    self.side_bar_sizer.Add(self.joint1_max_spin, 1, wx.CENTER | wx.TOP | wx.BOTTOM, 5)

    self.side_bar_sizer.Add(wx.StaticText(self, wx.ID_ANY, "Joint 2"), 1, wx.CENTER|wx.TOP, 5)
    self.link3_len_spin = wx.SpinCtrlDouble(self, wx.ID_ANY, initial=self.canvas.link3_length,
                                            min=-180, max=180)
    self.link3_len_spin.Bind(wx.EVT_SPINCTRLDOUBLE, self.update_kinematics)
    self.side_bar_sizer.Add(self.link3_len_spin, 1, wx.CENTER|wx.TOP|wx.BOTTOM, 5)

    self.joint2_min_spin = wx.SpinCtrlDouble(self, wx.ID_ANY, initial=np.degrees(self.canvas.joint2_min),
                                             min=-180, max=180)
    self.joint2_min_spin.Bind(wx.EVT_SPINCTRLDOUBLE, self.update_kinematics)
    self.side_bar_sizer.Add(self.joint2_min_spin, 1, wx.CENTER | wx.TOP | wx.BOTTOM, 5)

    self.joint2_max_spin = wx.SpinCtrlDouble(self, wx.ID_ANY, initial=np.degrees(self.canvas.joint2_max),
                                             min=-180, max=180)
    self.joint2_max_spin.Bind(wx.EVT_SPINCTRLDOUBLE, self.update_kinematics)
    self.side_bar_sizer.Add(self.joint2_max_spin, 1, wx.CENTER | wx.TOP | wx.BOTTOM, 5)

    self.side_bar_sizer.Add(wx.StaticLine(self, wx.ID_ANY, size=wx.Size(200, -1), style=wx.LI_HORIZONTAL))

    self.static_pose_ratio = wx.RadioButton(self, wx.ID_ANY, "Static Pose")
    self.static_pose_ratio.Enable()
    self.trajectory_ratio = wx.RadioButton(self, wx.ID_ANY, "Trajectory")
    self.dynamic_walk_ratio = wx.RadioButton(self, wx.ID_ANY, "Dynamic Walk")
    # self.static_pose_ratio.Bind(wx.EVT_RADIOBUTTON, self.view_mode_ratio_evt)
    # self.trajectory_ratio.Bind(wx.EVT_RADIOBUTTON, self.view_mode_ratio_evt)
    # self.dynamic_walk_ratio.Bind(wx.EVT_RADIOBUTTON, self.view_mode_ratio_evt)
    self.side_bar_sizer.Add(self.static_pose_ratio, 1, wx.LEFT, 20)
    self.side_bar_sizer.Add(self.trajectory_ratio, 1, wx.LEFT, 20)
    self.side_bar_sizer.Add(self.dynamic_walk_ratio, 1, wx.LEFT, 20)
    self.side_bar_sizer.Add(wx.StaticLine(self, wx.ID_ANY, size=wx.Size(200, -1), style=wx.LI_HORIZONTAL))
    self.set_view_mode(ViewMode.Static)

    icon = wx.Icon()
    icon.CopyFromBitmap(wx.Bitmap(os.path.join("..", "..", "resources", "bug_icon.png"), wx.BITMAP_TYPE_ANY))
    self.SetIcon(icon)

    self.Show()

    # self.animate_timer.Start(int(1000 / 30.0))

  def update_kinematics(self, _):
    self.canvas.joint1_min = np.radians(self.joint1_min_spin.GetValue())
    self.canvas.joint1_max = np.radians(self.joint1_max_spin.GetValue())
    self.canvas.joint2_min = np.radians(self.joint2_min_spin.GetValue())
    self.canvas.joint2_max = np.radians(self.joint2_max_spin.GetValue())

    self.canvas.link1_length = self.link1_len_spin.GetValue()
    self.canvas.link2_length = self.link2_len_spin.GetValue()
    self.canvas.link3_length = self.link3_len_spin.GetValue()
    self.canvas.update_kinematics()
    self.canvas.Refresh()

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

  # def on_open_trajectory_menu(self, _):
  #   print("Sommat")

  def on_quit_menu(self, _):
    self.Close()

  """def on_connect_robot(self, _):
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

      return traj"""


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
