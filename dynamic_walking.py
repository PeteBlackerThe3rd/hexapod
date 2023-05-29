"""
First attempt at a fully dynamic walking motion generator.

This uses the current robot state, and input velocity matrix to plan the next step and a half for each leg
and execute the next iteration of control
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRot


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
      :param vec:
      :param scalar:
      :return:
      """
      assert isinstance(subject, np.ndarray)

      # 2D vector case
      if subject.shape == (2,):
        pass

      # 3D vector case
      if subject.shape == (3,):
        pass

      # 4x4 tf matrix case
      if subject.shape == (4,4):
        tf = np.eye(4)
        rot_vec = [0, 0, self.rot_rate * scalar]
        tf[0:3, 0:3] = SciRot.from_rotvec(rot_vec).as_matrix()
        tf[3, 0:2] = [self.x_rate, self.y_rate] * scalar
        return subject * tf


class DynamicGait:

  STEP_DURATION = 1/15  # plan in 15ths of a second time steps

  def __init__(self, initial_joint_angles, kinematic_model):
    self.kin = kinematic_model
    self.joint_angles = initial_joint_angles
    self.current_toe_positions = [0] * 6

    self.toe_trajectories_this_step = None
    self.toe_trajectories_next_step = None

  def plan_using_body_velocity(self, velocity):

    # all leg planning is done in body space not world space, so the velocity we're
    # applying to the toe positions is the inverse of the robot velocity because
    # the toes need to be moving backwards!
    toe_vel = velocity.inverse()

    for leg_idx in range(6):
      # get current toe pos, assume there is a kinematic solution for where we are!
      toe_pos = self.current_toe_positions[leg_idx]
      traj_this_step = [toe_pos]

      # continue to propagate this toe position along the trajectory until there is no
      # longer a valid kinematic solution
      while self.kin.