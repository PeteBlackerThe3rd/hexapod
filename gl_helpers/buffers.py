from typing import Any
from OpenGL.GL import *
from OpenGL.GLUT import *


class VBO:
    def __init__(self, geom_type=GL_QUADS) -> None:
        self.vbo = glGenBuffers(1)
        self.component_count = 0
        self.vertex_count = 0
        self.geom_type = geom_type

    def delete(self) -> None:
        glDeleteBuffers(1, [self.vbo])

    def bind(self) -> None:
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)

    def unbind(self) -> None:
        glBindBuffer(GL_ARRAY_BUFFER, 0)

    def set_vertex_attribute(self, component_count: int, bytelength: int,
                             data: any) -> None:
        self.component_count = component_count
        stride = 4 * self.component_count
        self.vertex_count = bytelength // stride
        self.bind()
        glBufferData(GL_ARRAY_BUFFER, bytelength, data, GL_STATIC_DRAW)

    def set_slot(self, slot: int) -> None:
        self.bind()
        glEnableVertexAttribArray(slot)
        glVertexAttribPointer(slot, self.component_count, GL_FLOAT, GL_FALSE, 0, None)

    def draw(self) -> None:
        glDrawArrays(self.geom_type, 0, self.vertex_count)


class IBO:
    def __init__(self, geom_type=GL_QUADS) -> None:
        self.vbo = glGenBuffers(1)
        self.index_count = 0
        self.index_type = 0
        self.geom_type = geom_type

    def delete(self) -> None:
        glDeleteBuffers(1, [self.vbo])

    def bind(self) -> None:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.vbo)

    def unbind(self) -> None:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

    def set_indices(self, stride: int, bytelength: int, data: Any) -> None:
        self.index_count = bytelength // stride
        self.bind()
        if stride == 1:
            self.index_type = GL_UNSIGNED_BYTE
        elif stride == 2:
            self.index_type = GL_UNSIGNED_SHORT
        elif stride == 4:
            self.index_type = GL_UNSIGNED_INT
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, bytelength, data, GL_STATIC_DRAW)

    def draw(self) -> None:
        glDrawElements(self.geom_type, self.index_count, self.index_type, None)
