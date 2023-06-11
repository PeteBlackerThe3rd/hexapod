"""
First attempt at a fully dynamic walking motion generator.

This uses the current robot state, and input velocity matrix to plan the next step and a half for each leg
and execute the next iteration of control
"""
import numpy as np
import copy
from scipy.spatial.transform import Rotation as SciRot
import scipy.optimize
from translate_position import translate_datum, inverse_translate_datum
from leg_kinematics import NoKinematicSolution
from foot_velocities import Velocity2D, AnalyticFootTrajectory


class Velocity2D_old:
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
    centre_leg_dist_x = 0.165
    front_leg_dist_x = 0.105
    front_leg_dist_y = 0.127
    rear_leg_dist_x = 0.105
    rear_leg_dist_y = -0.127
    self.current_toe_positions = np.array([[front_leg_dist_x, front_leg_dist_y, -ride_height],
                                           [centre_leg_dist_x, 0, -ride_height],
                                           [rear_leg_dist_x, rear_leg_dist_y, -ride_height],
                                           [-rear_leg_dist_x, rear_leg_dist_y, -ride_height],
                                           [-centre_leg_dist_x, 0, -ride_height],
                                           [-front_leg_dist_x, front_leg_dist_y, -ride_height]])

    self.toe_trajectories_this_step = []
    self.toe_trajectories_next_step = []

    # generate working area line loops for each leg
    working_area = self.kin.compute_vector_working_area(-ride_height)
    working_area_boundary = working_area.get_boundary_points()
    self.working_area_boundaries = []
    self.working_area_boundary_point_count = working_area_boundary.shape[0]
    print("working_area_boundary_point_count = %d" % self.working_area_boundary_point_count)
    for leg_idx in range(6):
      leg_working_area_boundary = np.zeros(working_area_boundary.shape)
      for p_idx, point in enumerate(working_area_boundary):
        point_copy = copy.copy(point)
        leg_working_area_boundary[p_idx, :] = inverse_translate_datum(point_copy, leg_idx)
      self.working_area_boundaries.append(leg_working_area_boundary)

    self.last_toe_trajectory = None

  def propagate_toe_position(self, init_toe_pos, leg_idx, toe_vel, time_step):
    """
    Propagates the given toe position using the provided toe velocity, until there is
    no longer a kinematic solution for the given leg. Returns the time and distance
    until this point is reached along with a list of points along this trajectory
    :param init_toe_pos: 3D vector of toe position in robot bodt frame
    :param leg_idx: integer between 0-5
    :param toe_vel: Velocity Object describing the toe velocity described in the robot body frame
    :param time_step: float in seconds, the time duration of integration steps
    :return: tuple (duration, distance, list of points)
    """
    toe_pos = init_toe_pos
    traj_this_step = []
    total_duration = 0.0
    total_distance = 0.0
    max_steps = 1000
    max_distance = 0.1
    # continue to propagate this toe position along the trajectory until there is no
    # longer a valid kinematic solution
    for _ in range(max_steps):
      try:
        # print("toe_pos %s" % toe_pos)
        toe_pos_leg_frame = translate_datum(toe_pos, leg_idx)
        # print("toe pos leg frame: %s" % toe_pos_leg_frame)
        self.kin.inverse(toe_pos_leg_frame)
        # print("kinematic solution found")
        if len(traj_this_step) >= 1:
          total_duration += time_step
          total_distance += np.linalg.norm(toe_pos - traj_this_step[-1])
        traj_this_step.append(toe_pos)
        toe_pos = toe_vel.apply(toe_pos, time_step)
        if total_distance > max_distance:
          break
      except NoKinematicSolution:
        # print("No kinematic solution found")
        break

    self.last_toe_trajectory = traj_this_step
    return total_duration, total_distance, traj_this_step

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
      # traj_this_step = []

      # dynamically solve for a step duration which results in a toe movements of X m
      target_toe_movement = 0.5e-3  # half a mm

      def cost_fn(x):
        step = x[0]
        new_toe_pos = toe_vel.apply(toe_pos, step)
        delta = np.linalg.norm(new_toe_pos - toe_pos)
        error = abs(delta - target_toe_movement)
        # print("time_step %f, delta %f mm, error %f" % (x[0], delta * 1e3, error))
        return error

      result = scipy.optimize.minimize(cost_fn, (self.STEP_DURATION,), method='CG')
      opt_step_duration = abs(result.x[0])

      # if opt_step_duration < 0.0:
      #   print("Error negative step duration %f generated" % opt_step_duration)
      # print("step duration optimised from %f to %f seconds" % (self.STEP_DURATION, opt_step_duration))

      _, __, traj_this_step = self.propagate_toe_position(toe_pos, leg_idx, toe_vel, opt_step_duration)
      self.toe_trajectories_this_step.append(traj_this_step)

      # find optimal next step toe position
      """def next_step_cost_fn(x):
        _, next_step_distance, __ = self.propagate_toe_position(x, leg_idx, toe_vel, opt_step_duration)
        error = 0.2 - next_step_distance
        print("next_step_cost_fn (%s) [distance %f, error %f]" % (x, next_step_distance, error))
        return error

      std_dev = 0.05
      max_attempts_per_step = 20
      best_next_step = toe_pos
      best_dist = 0
      while True:
        improved = False
        for _ in range(max_attempts_per_step):
          toe_attempt = np.array([best_next_step[0] + np.random.normal(0, std_dev),
                                  best_next_step[1] + np.random.normal(0, std_dev),
                                  best_next_step[2]])
          _, next_step_distance, __ = self.propagate_toe_position(toe_attempt, leg_idx, toe_vel, opt_step_duration)
          if next_step_distance > best_dist:
            improved = True
            best_distance = best_dist
            best_next_step = toe_attempt
            break

        if not improved:
          break

      print("leg[%d] next toe optimized from [%s] to [%s]" % (leg_idx, toe_pos, best_next_step))
      _, distance, next_trajectory = self.propagate_toe_position(best_next_step, leg_idx, toe_vel, opt_step_duration)
      print("next toe [%s] distance to %f mm (%d stes)" % (best_next_step, distance * 1e3, len(next_trajectory)))
      self.toe_trajectories_next_step.append(next_trajectory)"""
