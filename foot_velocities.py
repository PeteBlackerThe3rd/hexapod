import numpy as np
import copy
from scipy.spatial.transform import Rotation as SciRot


class AnalyticFootTrajectory:
  """
  Class to store and operate on an analytical trajectory of a foot used by the free gait control algorithm.
  Stores either a linear velocity or rotational motion and can apply this to a point in the robot frame
  """
  LINEAR = 1
  CIRCULAR = 2

  def __init__(self, t_type, ):
    self.t_type = t_type
    self.linear_vel_m_s = np.zeros((2,))
    self.circular_centre = np.zeros((2,))
    self.circular_rate_rads_s = 0.0

  @classmethod
  def make_linear(cls, linear_vel):
    linear_traj = AnalyticFootTrajectory(cls.LINEAR)
    linear_traj.linear_vel_m_s = linear_vel
    return linear_traj

  @classmethod
  def make_circular(cls, centre, angular_rate):
    circular_traj = AnalyticFootTrajectory(cls.CIRCULAR)
    circular_traj.circular_centre = centre
    circular_traj.circular_rate_rads_s = angular_rate
    return circular_traj

  def propagate(self, point, time_secs):
    if self.t_type == self.LINEAR:
      return point + (self.linear_vel_m_s * time_secs)
    if self.t_type == self.CIRCULAR:
      angle = self.circular_rate_rads_s * time_secs
      rot_mat = np.array([[np.cos(angle), -np.sin(angle)],
                          [np.sin(angle), np.cos(angle)]])
      return np.matmul(rot_mat, point - self.circular_centre) + self.circular_centre


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

    def compute_foot_trajectory(self, pos):
      pos_x, pos_y = pos
      total_linear_vel = np.array((
        self.x_rate + (pos_x * np.cos(self.rot_rate)) - (pos_y * np.sin(self.rot_rate)) - pos_x,
        self.y_rate + (pos_y * np.sin(self.rot_rate)) + (pos_y * np.cos(self.rot_rate)) - pos_x,
      ))

      if self.rot_rate == 0.0:
        # print("trajectory is a staight line with velocity [%f, %f]" % (self.x_rate, self.y_rate))
        return AnalyticFootTrajectory.make_linear(total_linear_vel)
      else:
        radius = np.linalg.norm(total_linear_vel) / self.rot_rate
        if self.rot_rate > 0.0:
          cw_90 = np.array([[0, -1],
                            [1,  0]])
          centre_dir = np.matmul(cw_90, total_linear_vel / np.linalg.norm(total_linear_vel))
        else:
          ccw_90 = np.array([[0,  1],
                             [-1, 0]])
          centre_dir = np.matmul(ccw_90, total_linear_vel / np.linalg.norm(total_linear_vel))
        centre = pos + (centre_dir * radius)
        # print("trajectory is a circle of radius %f with centre [%f, %f]" % (radius, centre[0], centre[1]))
        return AnalyticFootTrajectory.make_circular(centre, -self.rot_rate)


def main():
  init_toe_pos = np.array([-0.1, 0.0, 0.0])
  toe_pos = copy.copy(init_toe_pos)

  vel = Velocity2D(0.4, 2.0, 0.1)
  inv_vel = vel.inverse()

  a_traj = inv_vel.compute_foot_trajectory((-0.1, 0.0))

  step_size = 1
  for step in range(2000):
    toe_pos = inv_vel.apply(np.array((toe_pos[0], toe_pos[1], toe_pos[2])), step_size)
    a_toe_pos = a_traj.propagate((-0.1, 0.0), (step+1) * step_size)
    print("%f, %f, , %f, %f" % (toe_pos[0], toe_pos[1], a_toe_pos[0], a_toe_pos[1]))


if __name__ == "__main__":
  main()
