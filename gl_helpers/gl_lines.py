from OpenGL.GLUT import *
from OpenGL.GL import *
import glm
from .shader import Shader
from .buffers import VBO, IBO


VS = '''
#version 330

uniform mat4 MVP;
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
out vec3 vsColor;

void main ()
{
    gl_Position = MVP * vec4(aPosition, 1);
    vsColor = aColor;
}
'''

FS = '''
#version 330

in vec3 vsColor;
out vec4 FragColor;

void main()
{
    FragColor = vec4(vsColor, 1);
}
'''


class GlLinesGeometry:
    def __init__(self):
        self.vbo = None
        self.vbo_col = None
        self.ibo = None
        self.shader = None
        self.mvp_location = None

        self.positions = []
        self.colors = []

        self.add_line([0, 0, 0], [1, 0, 0], 1, 0, 0)
        self.add_line([0, 0, 0], [0, 1, 0], 0, 1, 0)
        self.add_line([0, 0, 0], [0, 0, 1], 0, 0, 1)

    def add_line(self, start, end, r, g, b):

      color = [r, g, b]
      new_positions = list(start) + list(end)
      self.positions.extend(new_positions)
      self.colors.extend(color * 2)

    def initialize(self):
        self.shader = Shader()
        self.shader.compile(vs_src=VS, fs_src=FS)
        self.mvp_location = self.shader.get_uniform_location("MVP")
        self.update_geometry()

    def update_geometry(self):
        self.vbo = VBO(geom_type=GL_LINES)
        self.vbo_col = VBO()
        self.ibo = IBO(geom_type=GL_LINES)

        point_count = len(self.positions) // 3
        indices = range(point_count)

        sizeof_float = 4
        v_dims = 3

        self.vbo.set_vertex_attribute(component_count=v_dims,
                                      bytelength=sizeof_float * v_dims * point_count,
                                      data=(ctypes.c_float * (v_dims * point_count))(*self.positions))
        self.vbo_col.set_vertex_attribute(component_count=3,
                                          bytelength=sizeof_float * 3 * point_count,
                                          data=(ctypes.c_float * (3 * point_count))(*self.colors)
                                          )
        self.ibo.set_indices(stride=4,
                             bytelength=4 * point_count,
                             data=(ctypes.c_uint * point_count)(*indices))

    def draw(self, mvp):
      """
      Draw this gl_cloud to the viewport
      :param mvp: glmMat4 model view projection matrix.
      :return: None
      """
      if not self.vbo:
          self.initialize()
      self.shader.use()
      self.vbo.set_slot(0)
      self.vbo_col.set_slot(1)
      self.ibo.bind()
      glUniformMatrix4fv(self.mvp_location, 1, False, glm.value_ptr(mvp))
      self.ibo.draw()

      self.ibo.unbind()
      self.vbo.unbind()
      self.shader.unuse()
