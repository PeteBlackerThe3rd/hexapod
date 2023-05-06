"""

  PLY debug file
------------------

Helper class to build and save wire-frame models to PLY files which can be viewed using Meshlab and other tools

"""
import struct
import numpy as np


class PLYFile:

  def __init__(self):

    self.vertices = []
    self.vertex_colors = []
    self.lines = []

  def save(self, filename):
    """
    :param filename:
    :return:
    """
    header = "ply\n"
    header += "format binary_little_endian 1.0\n"
    header += "element vertex %d\n" % len(self.vertices)
    header += "property float x\nproperty float y\nproperty float z\n"
    header += "property uchar red\nproperty uchar green\nproperty uchar blue\n"
    header += "element edge %d\n" % len(self.lines)
    header += "property int vertex1\nproperty int vertex2\n"
    header += "end_header\n"

    file = open(filename, "wb")
    file.write(header.encode("utf-8"))

    for idx, v in enumerate(self.vertices):
      vc = self.vertex_colors[idx]
      file.write(struct.pack("<fffBBB", v[0], v[1], v[2], vc[0], vc[1], vc[2]))

    for line in self.lines:
      file.write(struct.pack("<II", line[0], line[1]))

  def add_point(self, p, r, g, b):
    self.vertices.append(np.array(p))
    self.vertex_colors.append((np.array([r, g, b])))

  def add_line(self, p1, p2, r, g, b):
    v1_idx = len(self.vertices)
    self.vertices.append(np.array(p1))
    self.vertices.append(np.array(p2))
    self.vertex_colors.append((np.array([r, g, b])))
    self.vertex_colors.append((np.array([r, g, b])))
    self.lines.append((v1_idx, v1_idx+1))
