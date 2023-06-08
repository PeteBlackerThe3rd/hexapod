import numpy as np
from scipy.spatial.transform import Rotation as SciRot


class AnalyticFootTrajectory:
  """
  Class to store and operate on an analytical trajectory of a foot used by the free gait control algorithm.
  Stores either a linear velocity or rotational motion.
  """
  LINEAR = 1
  CIRCULAR = 2

  def __init__(self, t_type, ):
    self.t_type = t_type
    self.linear_vel_m_s = np.zeros((2,))
    self.circular_centre = np.zeros((2,))
    self.circular_rate_rads_s = 0.0


class Velocity2D:
    """
    class used to represent the velocity of an object moving in 2 dimensions (X, Y, Rotation) rates
    rates are in m/s and rads/sec
    """

    def __init__(self, x_rate=0.0, y_rate=0.0, rot_rate=0.0):
      self.x_rate = x_rate
      self.y_rate = y_rate
      self.rot_rate = rot_rate

    def inverse(self):
      return Velocity2D(-self.x_rate, -self.y_rate, -self.rot_rate)

    def apply(self, subject, scalar):
      """
      Modifies the 2D/3D vector or 4x4 tf matrix subject as if this velocity has been applied for scalar Seconds.
      No integration is performed so only use this for small time steps
      :param subject: 2D/3D or 4x4 numpy.ndarray
      :param scalar: float value in seconds
      :return: same type as subject
      """
      assert isinstance(subject, np.ndarray)
      tf = np.eye(4)
      rot_vec = [0, 0, self.rot_rate * scalar]
      tf[0:3, 0:3] = SciRot.from_rotvec(rot_vec).as_matrix()
      tf[3, 0:2] = [self.x_rate * scalar, self.y_rate * scalar]

      # 2D vector case
      if subject.shape == (2,):
        vec = np.array([0, 0, 0, 1])
        vec[0:2] = subject
        result = np.matmul(vec, tf)
        return result[0:2]

      # 3D vector case
      if subject.shape == (3,):
        vec = np.ones(4,)
        vec[0:3] = subject
        result = np.matmul(vec, tf)
        return result[0:3]

      # 4x4 tf matrix case
      if subject.shape == (4, 4):
        return np.matmul(subject, tf)

    def is_zero(self):
      """
      checks if this velocity value is static
      :return: True if this velocity is zero, False otherwise
      """
      return self.x_rate == 0.0 and self.y_rate == 0.0 and self.rot_rate == 0.0

    def compute_foot_locus(self, pos):
      pos_x, pos_y = pos
      total = np.array((
        self.x_rate + (pos_x * np.cos(self.rot_rate)) - (pos_y * np.sin(self.rot_rate)) - pos_x,
        self.y_rate + (pos_y * np.sin(self.rot_rate)) + (pos_y * np.cos(self.rot_rate)) - pos_x,
      ))

      if self.rot_rate != 0:
        radius = np.linalg.norm(total) / (2.0 * np.tan(self.rot_rate/2))
        print("locus is a circle of radius %f" % radius)
      else:
        print("locus is a staight line with velocity [%f, %f]" % (self.x_rate, self.y_rate))


def main():
  toe_pos = np.array([-0.1, 0.0, 0.0])

  vel = Velocity2D(0.4, 2.0, 0.1)
  inv_vel = vel.inverse()

  scale = 1.0
  for _ in range(100):
    new_vel = Velocity2D(inv_vel.x_rate * scale, inv_vel.y_rate * scale, inv_vel.rot_rate * scale)
    print("scale = %f" % scale)
    inv_vel.compute_foot_locus((-0.1, 0.0))
    scale /= 2.0

  for _ in range(200):
    print("%f, %f" % (toe_pos[0], toe_pos[1]))
    toe_pos = inv_vel.apply(np.array((toe_pos[0], toe_pos[1], toe_pos[2])), 0.5)


if __name__ == "__main__":
  main()
