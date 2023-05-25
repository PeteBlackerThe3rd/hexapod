import math as m
import wx
import wx.glcanvas as glcanvas
from OpenGL.GL import *
import glm
import os
# from .gl_helpers.background_fade import BackgroundFade
# from .gl_helpers.gl_cloud import GlCloud
from .gl_lines import GlLinesGeometry
# from .gl_helpers.gl_polar_bounding_box import GlPolarBoundingBox
from .axes import GlAxes
from .actor import GLActor


class ViewerCanvas(glcanvas.GLCanvas):
    def __init__(self, parent):
        glcanvas.GLCanvas.__init__(self, parent, -1)
        self.context = glcanvas.GLContext(self)
        self.main_window = parent

        self.last_l_x = self.last_l_y = None
        self.last_m_x = self.last_m_y = None
        self.view_port_size = None
        self.dc = None

        self.view_centre = glm.vec3(0.0, 0.0, -10.8)
        self.view_scale = 25.0
        self.view_translation = glm.vec3(0.0, 0.0, 0.0)
        self.view_rotation = glm.mat4(1.0)

        # load robot link models
        self.link_actors = {"body": GLActor(os.path.join("resources", "models", "body_assy.stl"))}

        # Create GL helper objects used to display view elements
        self.axes = GlAxes(5)
        self.lines = GlLinesGeometry()
        self.lines_init = False

        self.tf_axes = GlAxes(0.03)

        self.SetBackgroundStyle(wx.BG_STYLE_PAINT)

        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_LEFT_DOWN, self.on_left_mouse_down)
        self.Bind(wx.EVT_LEFT_UP, self.on_left_mouse_up)
        self.Bind(wx.EVT_MIDDLE_DOWN, self.on_middle_mouse_down)
        self.Bind(wx.EVT_MIDDLE_UP, self.on_middle_mouse_up)
        self.Bind(wx.EVT_MOTION, self.OnMouseMotion)
        self.Bind(wx.EVT_MOUSEWHEEL, self.on_mouse_wheel)

    def update_robot_pose(self, positions):
        self.lines.clear_lines()

        # Add body hexagon
        for leg_idx in range(6):
            self.lines.add_line(positions[leg_idx]["joint_0"], positions[(leg_idx+1) % 6]["joint_0"], 1, 1, 1)

        # Add legs
        for leg_idx in range(6):
            self.lines.add_line(positions[leg_idx]["joint_0"], positions[leg_idx]["joint_1"], 1, 0, 0)
            self.lines.add_line(positions[leg_idx]["joint_1"], positions[leg_idx]["joint_2"], 0, 1, 0)
            self.lines.add_line(positions[leg_idx]["joint_2"], positions[leg_idx]["toe"], 0, 0, 1)

        self.lines.update_geometry()
        self.Refresh()

    def OnSize(self, event):
        wx.CallAfter(self.DoSetViewport)
        event.Skip()

    def DoSetViewport(self):
        self.view_port_size = self.GetClientSize()
        self.SetCurrent(self.context)
        glViewport(0, 0, self.view_port_size.width, self.view_port_size.height)
        self.Refresh()

    def OnPaint(self, _):
        self.dc = wx.PaintDC(self)
        self.SetCurrent(self.context)
        self.OnDraw()

    def on_left_mouse_down(self, evt):
        self.CaptureMouse()
        self.last_l_x, self.last_l_y = evt.GetPosition()

    def on_left_mouse_up(self, _):
        self.ReleaseMouse()

    def on_middle_mouse_down(self, evt):
        self.CaptureMouse()
        self.last_m_x, self.last_m_y = evt.GetPosition()

    def on_middle_mouse_up(self, _):
        self.ReleaseMouse()

    def get_arc_ball_vector(self, mouse_x, mouse_y):
        """
        Compute Arc Ball unit vector used by Arc Ball mouse drag behaviour.
        See : https://en.wikibooks.org/wiki/OpenGL_Programming/Modern_OpenGL_Tutorial_Arcball
        for explanation of this technique.
        :param mouse_x: Int, mouse x position in pixels
        :param mouse_y: Int, mouse y position in pixels
        :return: glm.Vec3, arcball unit vector
        """
        p = glm.vec3((float(mouse_x) / self.view_port_size.width) * 2 - 1,
                     0 - ((float(mouse_y) / self.view_port_size.height) * 2 - 1),
                     0)
        op_sq = p.x**2 + p.y**2
        if op_sq <= 1.0:
            p.z = m.sqrt(1 - op_sq)
        else:
            p = glm.normalize(p)
        return p

    def OnMouseMotion(self, evt):
        if self.last_l_x is None or self.last_l_y is None:
            return

        # For some reason on our dell mice LeftIsDown is true if only the middle is fully pressed!
        if evt.Dragging() and evt.LeftIsDown() and not evt.MiddleIsDown():
            new_x, new_y = evt.GetPosition()

            ab_vec_old = self.get_arc_ball_vector(self.last_l_x, self.last_l_y)
            ab_vec_new = self.get_arc_ball_vector(new_x, new_y)

            angle = m.acos(min(1.0, glm.dot(ab_vec_old, ab_vec_new)))
            axis_in_cam_space = glm.cross(ab_vec_old, ab_vec_new)
            axis_in_model_space = glm.inverse(glm.mat3(self.view_rotation)) * axis_in_cam_space

            # sanity check, to avoid bricking rotation vector.
            if glm.length2(axis_in_model_space) != 0:
                self.view_rotation = glm.rotate(self.view_rotation,
                                                angle,
                                                axis_in_model_space)

            self.last_l_x, self.last_l_y = new_x, new_y
            self.Refresh(False)

        elif evt.Dragging() and evt.MiddleIsDown():
            new_x, new_y = evt.GetPosition()
            factor = 0.01 / self.view_scale
            delta_screen = glm.vec3((new_x - self.last_m_x) * factor,
                                    (self.last_m_y - new_y) * factor,  # Note OpenGL y axis is inverted from GUI
                                    0.0)
            delta_model = glm.inverse(glm.mat3(self.view_rotation)) * delta_screen
            self.view_translation += delta_model

            self.last_m_x, self.last_m_y = new_x, new_y
            self.Refresh(False)

    def on_mouse_wheel(self, evt):
        """
        Handle mouse wheel events by scaling the point cloud up and down
        :param evt: wx.Event
        :return: None
        """
        factor = 0.1

        wheel = evt.GetWheelRotation() / float(evt.GetWheelDelta())
        if wheel > 0:
            self.view_scale *= 1.0 + (factor * wheel)
        else:
            self.view_scale /= 1.0 + (factor * -wheel)
        self.Refresh(False)
        print("Updated view_scale to %f" % self.view_scale)

    def set_trajecory(self,):
        """
        outline function to update the trajectory being viewed
        :return: None
        """
        # do stuff
        self.Refresh(False)

    def OnDraw(self):
        """
        Render the openGL point view panel
        :return: None
        """
        if not self.lines_init:
            self.lines.initialize()
            print("init lines object")
            self.lines_init = True

        # clear viewport to black
        glViewport(0, 0, self.view_port_size.width, self.view_port_size.height)
        glClearColor(0.0, 0.0, 0.0, GL_DEPTH_CLEAR_VALUE)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # create model view projection matrix
        aspect_ratio = self.view_port_size.width / self.view_port_size.height
        view_centered = glm.translate(glm.mat4(1.0), self.view_centre)
        view_translated = glm.translate(view_centered,
                                        glm.mat3(self.view_rotation) * (self.view_translation * self.view_scale))
        view_scaled = glm.scale(view_translated, glm.vec3(self.view_scale))
        model_view = view_scaled * self.view_rotation
        min_depth = 0.1  # 1 millimeters
        max_depth = 20.0   # 100 meters
        projection = glm.perspective(45.0, aspect_ratio, min_depth, max_depth)
        mvp = projection * model_view

        # glLineWidth(3.0)
        # self.lines.draw(mvp)

        # draw axes for each frame if enabled
        for label, frame in self.main_window.frames.items():
            frame_mvp = mvp * glm.mat4(frame)
            self.tf_axes.draw(frame_mvp)

        # draw all links of the robot in their respective coordinate frames
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        for frame, actor in self.link_actors.items():
            actor.draw(mvp)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_CULL_FACE)

        # setup axes viewport
        axes_size = min(self.view_port_size.width, self.view_port_size.height) // 5
        glViewport(self.view_port_size.width - axes_size, 0,
                   axes_size, axes_size)

        # setup axes view projection matrix
        projection = glm.perspective(45.0, 1.0, 0.1, 20.0)
        axes_location = glm.translate(glm.mat4(1.0), glm.vec3(0, 0, -1.2))
        vp = projection * glm.scale(axes_location, glm.vec3(0.1)) * self.view_rotation

        # render small axes
        glClearDepth(GL_DEPTH_CLEAR_VALUE)
        glClear(GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)
        self.axes.draw(vp)

        self.SwapBuffers()
