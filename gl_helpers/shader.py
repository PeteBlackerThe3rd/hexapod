
from OpenGL.GL import *
from OpenGL.GLUT import *


def load_shader(src: str, shader_type: int) -> int:
    shader = glCreateShader(shader_type)
    glShaderSource(shader, src)
    glCompileShader(shader)
    error = glGetShaderiv(shader, GL_COMPILE_STATUS)
    if error != GL_TRUE:
        info = glGetShaderInfoLog(shader)
        glDeleteShader(shader)
        raise Exception(info)
    return shader


class Shader:
    def __init__(self) -> None:
        self.program = glCreateProgram()

    def delete(self) -> None:
        glDeleteProgram(self.program)

    def compile(self, vs_src, fs_src, gs_src=None):
        vs = load_shader(vs_src, GL_VERTEX_SHADER)
        if not vs:
            return
        fs = load_shader(fs_src, GL_FRAGMENT_SHADER)
        if not fs:
            return
        if gs_src is not None:
            gs = load_shader(gs_src, GL_GEOMETRY_SHADER)
        else:
            gs = None
        glAttachShader(self.program, vs)
        glAttachShader(self.program, fs)
        if gs is not None:
            glAttachShader(self.program, gs)
        glLinkProgram(self.program)
        error = glGetProgramiv(self.program, GL_LINK_STATUS)
        glDeleteShader(vs)
        glDeleteShader(fs)
        if gs is not None:
            glDeleteShader(gs)
        if error != GL_TRUE:
            info = glGetShaderInfoLog(self.program)
            raise Exception(info)

    def get_uniform_location(self, uniform_label):
        return glGetUniformLocation(self.program, uniform_label)

    def use(self):
        glUseProgram(self.program)

    def unuse(self):
        glUseProgram(0)
