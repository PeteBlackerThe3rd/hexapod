"""
  First attempt at generating a walking gait
----------------------------------------------

  Uses a multivariate solver and a scalar cost function to find the
  trajectory with the largest forwards motion for a single step per leg walk.

"""
import numpy as np
from ply_debug_file import PLYFile


def generate_all_leg_working_area(height, filename):
  """

  :param height:
  :return:
  """
  pass


def linear_bezier(p0, p1, ratio):
  return (p0 * (1-ratio)) + (p1 * ratio)


def quadratic_bezier(p0, p1, p2, ratio):
  p01 = linear_bezier(p0, p1, ratio)
  p12 = linear_bezier(p1, p2, ratio)
  return (p01 * (1-ratio)) + (p12 * ratio)


def cubic_bezier(p0, p1, p2, p3, ratio):
  p012 = quadratic_bezier(p0, p1, p2, ratio)
  p123 = quadratic_bezier(p1, p2, p3, ratio)
  return (p012 * (1-ratio)) + (p123 * ratio)


class TrajectoryPoint:
  def __init__(self, time, position):
    self.time = time
    self.pos = position


class Trajectory:
  def __init__(self):
    self.points = []

  def append(self, t, pos):
    self.points.append(TrajectoryPoint(t, pos))

  def extend(self, new_traj):
    t_offset = self.points[-1].time
    for point in new_traj.points:
      self.points.append(TrajectoryPoint(point.time + t_offset, point.pos))

  def interpolate_even_timesteps(self, step_count):
    time_array = []
    x_array = []
    y_array = []
    for point in self.points:
      time_array.append(point.time)
      x_array.append(point.pos[0])
      y_array.append(point.pos[1])

    new_time_array = np.linspace(0, self.points[-1].time, step_count)
    new_x_array = np.interp(new_time_array, time_array, x_array)
    new_y_array = np.interp(new_time_array, time_array, y_array)

    new_traj = Trajectory()
    for idx, time in enumerate(new_time_array):
      new_traj.append(time, np.array([new_x_array[idx], new_y_array[idx]]))

    return new_traj


def gen_walking_toe_trajectory():
  """
  Use cubic bezier splines to create a trajectory where the toe
  smoothly transitions from moving on the ground to lifting
  and moving back to a new contact point with the ground.
  Use a cosine time shift to be able to control the duration
  of time in contact with the floor and time performing the lift
  :return: 1 cycle of the smooth toe trajectory over 1 second
  """
  floor_traj = Trajectory()
  lift_traj = Trajectory()

  step_start = np.array([-0.7, 0])
  step_end = np.array([0.7, 0])
  step_start_control = np.array([-1.1, 0])
  step_end_control = np.array([1.1, 0])
  lift = np.array([0, 0.4])
  lift_start_control = np.array([0.7, 0.4])
  lift_end_control = np.array([-0.7, 0.4])

  floor_duration = 0.50
  lift_duration = 0.50

  # create linear floor trajectory so robot moves at constant speed
  for ratio in np.linspace(0, 1, 100):
    floor_traj.append(ratio * floor_duration, linear_bezier(step_start, step_end, ratio))
  floor_vel = np.linalg.norm(step_end - step_start) / floor_duration

  print("Floor traj duration = %f" % floor_duration)

  # create the lift trajectory where time = distance / floor_vel
  dist_sum = 0
  last_p = step_end
  for ratio in np.linspace(0, 1, 100):
    next_p = cubic_bezier(step_end, step_end_control, lift_start_control, lift, ratio)
    dist_sum += np.linalg.norm(next_p - last_p)
    lift_traj.append(dist_sum / floor_vel, next_p)
    last_p = next_p

  for ratio in np.linspace(0, 1, 100):
    next_p = cubic_bezier(lift, lift_end_control, step_start_control, step_start, ratio)
    dist_sum += np.linalg.norm(next_p - last_p)
    lift_traj.append(dist_sum / floor_vel, next_p)
    last_p = next_p

  lift_initial_duration = lift_traj.points[-1].time
  print("initial lift trajectory duration = %f" % lift_initial_duration)

  # alter duration of lift trajectory so it is exactly 0.45 using a sine wave to preserve initial and final velocities
  duration_delta = lift_duration - lift_initial_duration

  time_scale = np.pi / lift_initial_duration
  for point in lift_traj.points:
    point.time += ((np.cos(np.pi + (point.time * time_scale)) + 1) / 2) * duration_delta

  lift_final_duration = lift_traj.points[-1].time
  print("final lift trajectory duration = %f" % lift_final_duration)

  floor_traj.extend(lift_traj)

  even_traj = floor_traj.interpolate_even_timesteps(200)

  debug = PLYFile()

  # for point in floor_traj.points:
  #   debug.add_point(np.array([point.time, point.pos[0], point.pos[1]]), 255, 0, 0)

  for point in even_traj.points:
    debug.add_point(np.array([point.time + 0, point.pos[0], point.pos[1]]), 255, 255, 0)
    debug.add_point(np.array([point.time + 1, point.pos[0], point.pos[1]]), 255, 255, 0)
    debug.add_point(np.array([point.time + 2, point.pos[0], point.pos[1]]), 255, 255, 0)

  # for point in lift_traj.points:
  #   debug.add_point(np.array([point.time + floor_traj.points[-1].time, point.pos[0], point.pos[1]]), 255, 255, 0)

  debug.save("toe_trajectory.ply")
  return even_traj


if __name__ == "__main__":
  gen_walking_toe_trajectory()
