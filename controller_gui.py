"""

  Hexapod Control GUI
 ---------------------

"""
import os
import wx
from gl_helpers.viewer_canvas import ViewerCanvas
from gait_generator import optimise_walking_gait
from translate_position import joints_to_all_leg_positions


class MainMenu(wx.MenuBar):

  ID_OPEN = 1001
  ID_QUIT = 1010

  ID_GEN_WALKING = 2001

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
    self.gen_menu.Append(self.ID_GEN_WALKING, "Generator Walking Gait")
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
    self.menu_bar.bind_menu_handler(self.menu_bar.ID_GEN_WALKING, self.gen_walking_gait)

    window_sizer = wx.BoxSizer(wx.HORIZONTAL)
    self.SetSizer(window_sizer)

    self.joint_trajectory = None
    self.animation_step = 0
    self.animate_timer = wx.Timer(self)
    self.Bind(wx.EVT_TIMER, self.update_frame, self.animate_timer)

    self.canvas = ViewerCanvas(self)
    window_sizer.Add(self.canvas, proportion=2, flag=wx.EXPAND)

    icon = wx.Icon()
    icon.CopyFromBitmap(wx.Bitmap(os.path.join("resources", "bug_icon.png"), wx.BITMAP_TYPE_ANY))
    self.SetIcon(icon)

    self.Show()

  def on_open_trajectory_menu(self, _):
    print("Sommat")

  def on_quit_menu(self, _):
    self.Close()

  def gen_walking_gait(self, _):
      self.joint_trajectory = optimise_walking_gait()
      print(self.joint_trajectory)
      # positions = joints_to_all_leg_positions(joint_trajectory[0])
      self.animate_timer.Start(15)
      # self.canvas.update_robot_pose(positions)

  def update_frame(self, _):
    self.animation_step = (self.animation_step + 1) % len(self.joint_trajectory)
    positions = joints_to_all_leg_positions(self.joint_trajectory[self.animation_step])
    self.canvas.update_robot_pose(positions)


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
