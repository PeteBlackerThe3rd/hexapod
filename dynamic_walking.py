"""
First attempt at a fully dynamic walking motion generator.

This uses the current robot state, and input velocity matrix to plan the next step and a half for each leg
and execute the next iteration of control
"""
import numpy as np
from scipy.spatial.transform import Rotation as SciRot
from translate_position import translate_datum
from leg_kinematics import NoKinematicSolution


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
        pass

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


class DynamicGait:

  STEP_DURATION = 1/15  # plan in 15ths of a second time steps

  def __init__(self, kinematic_model):
    self.kin = kinematic_model

    # self.base_frames = get_leg_base_frames()

    # crude way of setting initial toe positions
    ride_height = 0.08
    centre_leg_dist_x = 0.17
    front_leg_dist_x = 0.115
    front_leg_dist_y = 0.137
    rear_leg_dist_x = 0.115
    rear_leg_dist_y = -0.137
    self.current_toe_positions = np.array([[front_leg_dist_x, front_leg_dist_y, -ride_height],
                                           [centre_leg_dist_x, 0, -ride_height],
                                           [rear_leg_dist_x, rear_leg_dist_y, -ride_height],
                                           [-rear_leg_dist_x, rear_leg_dist_y, -ride_height],
                                           [-centre_leg_dist_x, 0, -ride_height],
                                           [-front_leg_dist_x, front_leg_dist_y, -ride_height]])

    self.toe_trajectories_this_step = []
    self.toe_trajectories_next_step = []

  def plan_using_body_velocity(self, velocity):

    # all leg planning is done in body space not world space, so the velocity we're
    # applying to the toe positions is the inverse of the robot velocity because
    # the toes need to be moving backwards!
    toe_vel = velocity.inverse()

    max_steps = int(5 / self.STEP_DURATION)

    self.toe_trajectories_this_step = []
    self.toe_trajectories_next_step = []

    # for now ignore the case where velocity is zero (this means that things need to be done differently!)
    if velocity.is_zero():
      return

    for leg_idx in range(6):
      # get current toe pos, assume there is a kinematic solution for where we are!
      toe_pos = self.current_toe_positions[leg_idx]
      # print("leg [%d] initial toe_pos %s" % (leg_idx, toe_pos))
      traj_this_step = []

      # continue to propagate this toe position along the trajectory until there is no
      # longer a valid kinematic solution
      for _ in range(max_steps):
        try:
          #print("toe_pos %s" % toe_pos)
          toe_pos_leg_frame = translate_datum(toe_pos, leg_idx)
          #print("toe pos leg frame: %s" % toe_pos_leg_frame)
          self.kin.inverse(toe_pos_leg_frame)
          #print("kinematic solution found")
          traj_this_step.append(toe_pos)
          toe_pos = toe_vel.apply(toe_pos, self.STEP_DURATION)
        except NoKinematicSolution:
          #print("No kinematic solution found")
          break

      # print("leg [%d] completed with %d steps" % (leg_idx, len(traj_this_step)))

      self.toe_trajectories_this_step.append(traj_this_step)
