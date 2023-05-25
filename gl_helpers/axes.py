import math as m
import numpy as np
from OpenGL.GL import *
import glm
from .shader import Shader
from .buffers import VBO, IBO


VS = '''
#version 330

uniform mat4 MVP;
layout(location = 0) in vec3 vPos;
layout(location = 1) in vec3 vCol;
layout(location = 2) in vec3 vNor;

out vec3 color;
out vec3 position;

void main()
{
    // Apply MVP matrix and pass vertex location down.
    vec4 pos_4 = MVP * vec4(vPos, 1.0);
    gl_Position = pos_4;
    
    // Apply MVP matrix rotation only to normal
    vec4 viewSpaceNor4 = MVP * vec4(vNor, 0);
    vec3 viewSpaceNor = normalize(viewSpaceNor4.xyz);
    
    // define light direction unit vector
    vec3 light = vec3(1.0, -1.0, 1.0);
    light = normalize(light);
    
    // Compute incident angle of light
    float dot_t = dot(viewSpaceNor, light);
    float i = acos(dot_t) / 3.141592654;
    
    // Apply diffuse shading to color
    color = vec3(i * vCol.x, i * vCol.y, i * vCol.z);
    
    // Apply specular highlight to lighting
    float i_spec = max(0.0, (i*4.0) - 3.0);
    float i_spec_1 = 1 - i_spec;
    color.x = (color.x * i_spec_1) + i_spec;
    color.y = (color.y * i_spec_1) + i_spec;
    color.z = (color.z * i_spec_1) + i_spec;
}
'''

FS = '''
#version 330

in vec3 color;
in vec3 position;

out vec4 FragColor;

void main(void)
{
    FragColor = vec4(color[0], color[1], color[2], 1.0);
}
'''


class GlAxes:
    def __init__(self, scale=1.0):
        self.vbo = None
        self.vbo_col = None
        self.vbo_norm = None
        self.ibo = None
        self.shader = None
        self.mvp_location = None
        self.initialized = False

        self.vertices = []
        self.normals = []
        self.colors = []
        self.v_count = 0

        self.scale = scale * 0.2

        # Add axis arrows in X, Y, and Z directions
        self.add_axis(5.0, [1, 0, 0], glm.mat4(1.0))
        trans = glm.rotate(glm.mat4(1.0), m.pi / 2, glm.vec3(0, 0, 1))
        self.add_axis(5.0, [0, 1, 0], trans)
        trans = glm.rotate(glm.mat4(1.0), m.pi / 2, glm.vec3(0, -1, 0))
        self.add_axis(5.0, [0.3, 0.3, 1], trans)
        self.indices = range(self.v_count)

    def add_triangle(self, v1, v2, v3, color):

      # compute normal
      normal = glm.normalize(glm.cross((v1 - v2).xyz, (v2 - v3).xyz))
      self.vertices.extend([v1.x * self.scale, v1.y * self.scale, v1.z * self.scale])
      self.vertices.extend([v2.x * self.scale, v2.y * self.scale, v2.z * self.scale])
      self.vertices.extend([v3.x * self.scale, v3.y * self.scale, v3.z * self.scale])
      self.colors.extend(color)
      self.colors.extend(color)
      self.colors.extend(color)
      self.normals.extend([normal.x, normal.y, normal.z])
      self.normals.extend([normal.x, normal.y, normal.z])
      self.normals.extend([normal.x, normal.y, normal.z])
      self.v_count += 3

    def add_axis(self, size, color, trans):
      """
      Method to add an axis arrow to the geometry lists of this objet
      :param size: The length of the axis arrow
      :param color: 3 element list, the color in RGB float
      :param trans: glm.mat4, a translation matrix to apply to the generated axis
      :return: None
      """
      segments = 64
      head_length = size * 0.25
      shaft_length = size - head_length
      shaft_diameter = size * 0.05
      head_diameter = size * 0.1

      root = trans * glm.vec4(0, 0, 0, 0)
      tip = trans * glm.vec4(size, 0, 0, 0)

      for a in np.linspace(0, 2 * m.pi, segments, endpoint=False):

        # Compute vertices around arrow
        a_last = a - (2 * m.pi) / segments
        base = trans * glm.vec4(0, m.sin(a) * shaft_diameter, m.cos(a) * shaft_diameter, 0)
        base_l = trans * glm.vec4(0, m.sin(a_last) * shaft_diameter, m.cos(a_last) * shaft_diameter, 0)
        head_inner = trans * glm.vec4(shaft_length, m.sin(a) * shaft_diameter, m.cos(a) * shaft_diameter, 0)
        head_inner_l = trans * glm.vec4(shaft_length, m.sin(a_last) * shaft_diameter, m.cos(a_last) * shaft_diameter, 0)
        head_outer = trans * glm.vec4(shaft_length, m.sin(a) * head_diameter, m.cos(a) * head_diameter, 0)
        head_outer_l = trans * glm.vec4(shaft_length, m.sin(a_last) * head_diameter, m.cos(a_last) * head_diameter, 0)

        self.add_triangle(base, root, base_l, color)

        self.add_triangle(base, base_l, head_inner, color)
        self.add_triangle(head_inner, base_l, head_inner_l, color)

        self.add_triangle(head_inner, head_inner_l, head_outer, color)
        self.add_triangle(head_outer, head_inner_l, head_outer_l, color)

        self.add_triangle(head_outer, head_outer_l, tip, color)

    def initialize(self):
        self.shader = Shader()
        self.shader.compile(vs_src=VS, fs_src=FS)
        self.mvp_location = self.shader.get_uniform_location("MVP")
        self.vbo = VBO(geom_type=GL_TRIANGLES)
        self.vbo_col = VBO()
        self.vbo_norm = VBO()
        self.ibo = IBO(geom_type=GL_TRIANGLES)

        sizeof_float = 4
        v_dims = n_dims = 3

        print("Axes init:\nv_count:%d\nvertices len:%d\ncolors len:%d\nnormals len:%d"% (
          self.v_count,
          len(self.vertices),
          len(self.colors),
          len(self.normals)
        ))

        self.vbo.set_vertex_attribute(component_count=v_dims,
                                      bytelength=sizeof_float * v_dims * self.v_count,
                                      data=(ctypes.c_float * (v_dims * self.v_count))(*self.vertices))
        self.vbo_col.set_vertex_attribute(component_count=3,
                                          bytelength=sizeof_float * 3 * self.v_count,
                                          data=(ctypes.c_float * (3 * self.v_count))(*self.colors)
                                          )
        self.vbo_norm.set_vertex_attribute(component_count=n_dims,
                                           bytelength=sizeof_float * n_dims * self.v_count,
                                           data=(ctypes.c_float * (n_dims * self.v_count))(*self.normals))
        self.ibo.set_indices(stride=4,
                             bytelength=4 * self.v_count,
                             data=(ctypes.c_uint * self.v_count)(*self.indices))

    def draw(self, mvp):
      """
      Draw this gl_cloud to the viewport
      :param mvp: glmMat4 model view projection matrix.
      :return: None
      """
      # glEnable(GL_CULL_FACE)
      if not self.initialized:
          self.initialize()
          self.initialized = True
      self.shader.use()
      self.vbo.set_slot(0)
      self.vbo_col.set_slot(1)
      self.vbo_norm.set_slot(2)
      self.ibo.bind()
      glUniformMatrix4fv(self.mvp_location, 1, False, glm.value_ptr(mvp))
      self.ibo.draw()

      self.ibo.unbind()
      self.vbo.unbind()
      self.shader.unuse()
