"""
  First attempt at generating a walking gait
----------------------------------------------

  Uses a multivariate solver and a scalar cost function to find the
  trajectory with the largest forwards motion for a single step per leg walk.

"""
import numpy as np
from ply_debug_file import PLYFile
from translate_position import translate_datum, inverse_translate_datum
from leg_kinematics import LegKinematics, NoKinematicSolution

def generate_all_leg_index_test(filename):
  """
  :param filename:
  :return:
  """
  debug_file = PLYFile()

  red = [255, 0, 0, 128, 0, 0]
  green = [0, 255, 0, 0, 128, 0]
  blue = [0, 0, 255, 0, 0, 128]

  for idx in range(6):
    translated_root = inverse_translate_datum(np.zeros((3,)), idx)
    debug_file.add_point(translated_root, red[idx], green[idx], blue[idx])
    translated_toe = inverse_translate_datum(np.array([0, 0.08, 0]), idx)
    debug_file.add_point(translated_toe, red[idx], green[idx], blue[idx])

    # orig_root = translate_datum

  debug_file.save(filename)


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


def gen_walking_toe_trajectory(floor_distance=1.4, lift_height=0.4, floor_duration=0.5, save=False):
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

  step_start = np.array([-floor_distance*0.5, 0])
  step_end = np.array([floor_distance*0.5, 0])
  step_start_control = np.array([-floor_distance*0.75, 0])
  step_end_control = np.array([floor_distance*0.75, 0])
  lift = np.array([0, lift_height])
  lift_start_control = np.array([floor_distance*0.5, lift_height])
  lift_end_control = np.array([-floor_distance*0.5, lift_height])

  lift_duration = 1 - floor_duration

  # create linear floor trajectory so robot moves at constant speed
  for ratio in np.linspace(0, 1, 100):
    floor_traj.append(ratio * floor_duration, linear_bezier(step_start, step_end, ratio))
  floor_vel = np.linalg.norm(step_end - step_start) / floor_duration

  # print("Floor traj duration = %f" % floor_duration)

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
  # print("initial lift trajectory duration = %f" % lift_initial_duration)

  # alter duration of lift trajectory so it is exactly 0.45 using a sine wave to preserve initial and final velocities
  duration_delta = lift_duration - lift_initial_duration

  time_scale = np.pi / lift_initial_duration
  for point in lift_traj.points:
    point.time += ((np.cos(np.pi + (point.time * time_scale)) + 1) / 2) * duration_delta

  # lift_final_duration = lift_traj.points[-1].time
  # print("final lift trajectory duration = %f" % lift_final_duration)

  floor_traj.extend(lift_traj)
  even_traj = floor_traj.interpolate_even_timesteps(200)

  if save:
    debug = PLYFile()
    for point in even_traj.points:
      debug.add_point(np.array([point.time + 0, point.pos[0], point.pos[1]]), 255, 255, 0)
      debug.add_point(np.array([point.time + 1, point.pos[0], point.pos[1]]), 255, 255, 0)
      debug.add_point(np.array([point.time + 2, point.pos[0], point.pos[1]]), 255, 255, 0)
    debug.save("toe_trajectory.ply")

  return even_traj


def cost_fn(state_vector, debug_filename=None):
  """
  Cost function of the walking gait optimisation.
  returns the distance moved forwards or None is no kinematic solution exists
  :param state_vector:
  :param debug_filename
  :return: cost value
  """
  ride_height, floor_length, centre_leg_dist_x, front_leg_dist_x, front_left_dist_y, \
      rear_leg_dist_x, rear_left_dist_y = state_vector

  # create toe trajectory based upon ride height and floor length
  toe_traj = gen_walking_toe_trajectory(floor_distance=floor_length, lift_height=0.02, floor_duration=0.5)

  kin = LegKinematics()
  if debug_filename is not None:
    debug_file = PLYFile()

  # check kinematic solutions exist for whole trajectory for both leg classes
  failed = False
  for toe_p in toe_traj.points:
    centre_toe_p = np.array([centre_leg_dist_x, toe_p.pos[0], toe_p.pos[1] - ride_height])
    front_toe_p = np.array([front_leg_dist_x, toe_p.pos[0] + front_left_dist_y, toe_p.pos[1] - ride_height])
    rear_toe_p = np.array([rear_leg_dist_x, toe_p.pos[0] + rear_left_dist_y, toe_p.pos[1] - ride_height])

    front_leg_p = translate_datum(front_toe_p, 0)
    centre_leg_p = translate_datum(centre_toe_p, 1)
    rear_leg_p = translate_datum(rear_toe_p, 2)
    try:
      kin.inverse(centre_leg_p)
      if debug_filename is not None:
        debug_file.add_point(centre_toe_p, 255, 255, 255)
    except NoKinematicSolution:
      if debug_filename is not None:
        debug_file.add_point(centre_toe_p, 255, 0, 0)
      failed = True

    try:
      kin.inverse(front_leg_p)
      if debug_filename is not None:
        debug_file.add_point(front_toe_p, 255, 255, 255)
    except NoKinematicSolution:
      if debug_filename is not None:
        debug_file.add_point(front_toe_p, 255, 0, 0)
      failed = True

    try:
      kin.inverse(rear_leg_p)
      if debug_filename is not None:
        debug_file.add_point(rear_toe_p, 255, 255, 255)
    except NoKinematicSolution:
      if debug_filename is not None:
        debug_file.add_point(rear_toe_p, 255, 0, 0)
      failed = True

  if debug_filename is not None:
    debug_file.save(debug_filename)

  if failed:
    return None
  else:
    return floor_length


def create_gait_trajectory(state_vector, steps_per_sec, debug_filename=None):
  """
  Create a full robot joint trajectory for the walking gait described by the state vector
  :param state_vector:
  :param debug_filename
  :return: cost value
  """
  ride_height, floor_length, centre_leg_dist_x, front_leg_dist_x, front_left_dist_y, \
      rear_leg_dist_x, rear_left_dist_y = state_vector

  # create toe trajectory based upon ride height and floor length
  toe_traj = gen_walking_toe_trajectory(floor_distance=floor_length, lift_height=0.02, floor_duration=0.5)

  red = [255, 0, 0, 128, 0, 0]
  green = [0, 255, 0, 0, 128, 0]
  blue = [0, 0, 255, 0, 0, 128]

  traj_length = len(toe_traj.points)
  half_length = traj_length // 2
  leg_toe_trajectories = {0: np.zeros((traj_length, 3)),
                          1: np.zeros((traj_length, 3)),
                          2: np.zeros((traj_length, 3)),
                          3: np.zeros((traj_length, 3)),
                          4: np.zeros((traj_length, 3)),
                          5: np.zeros((traj_length, 3))}
  joint_trajectory = np.zeros((traj_length, 18))

  kin = LegKinematics()
  if debug_filename is not None:
    debug_file = PLYFile()

  for idx, toe_p in enumerate(toe_traj.points):
    idx_opp = (idx + half_length) % traj_length
    leg_toe_trajectories[0][idx, :] = [front_leg_dist_x, toe_p.pos[0] + front_left_dist_y, toe_p.pos[1] - ride_height]
    leg_toe_trajectories[1][idx_opp, :] = [centre_leg_dist_x, toe_p.pos[0], toe_p.pos[1] - ride_height]
    leg_toe_trajectories[2][idx, :] = [rear_leg_dist_x, toe_p.pos[0] + rear_left_dist_y, toe_p.pos[1] - ride_height]
    leg_toe_trajectories[5][idx_opp, :] = [-front_leg_dist_x, toe_p.pos[0] + front_left_dist_y, toe_p.pos[1] - ride_height]
    leg_toe_trajectories[4][idx, :] = [-centre_leg_dist_x, toe_p.pos[0], toe_p.pos[1] - ride_height]
    leg_toe_trajectories[3][idx_opp, :] = [-rear_leg_dist_x, toe_p.pos[0] + rear_left_dist_y, toe_p.pos[1] - ride_height]

  floor_vel = floor_length / 0.5
  for joint in [0, 1, 2, 3, 4, 5]:  # range(6):
    for idx in range(traj_length * 3):
      goal_leg_space = translate_datum(leg_toe_trajectories[joint][(idx % traj_length), :], joint)
      try:
        joint_angles = kin.inverse(goal_leg_space)
        joint_trajectory[(idx % traj_length), (joint*3):(joint*3+3)] = joint_angles

        if debug_filename is not None:

          time = idx / 200.0
          floor_motion = np.array([0, -floor_vel * time, 0])

          joint_positions_leg_space = kin.forwards_all_joints(joint_angles)
          for key, pos in joint_positions_leg_space.items():
            pos = inverse_translate_datum(pos, joint)
            joint_positions_leg_space[key] = pos + floor_motion
          debug_file.add_line(joint_positions_leg_space["joint_0"], joint_positions_leg_space["joint_1"], red[joint], green[joint], blue[joint])
          debug_file.add_line(joint_positions_leg_space["joint_1"], joint_positions_leg_space["joint_2"], red[joint], green[joint], blue[joint])
          debug_file.add_line(joint_positions_leg_space["joint_2"], joint_positions_leg_space["toe"], red[joint], green[joint], blue[joint])
      except NoKinematicSolution:
        toe_pos_debug = leg_toe_trajectories[joint][idx, :] + np.array([0, 0, -0.005])
        debug_file.add_point(toe_pos_debug, 255, 0, 0)
        pass

  if debug_filename is not None:
    debug_file.save(debug_filename)

  return joint_trajectory


def optimise_walking_gait():

  # define seed values
  ride_height = 0.04
  floor_length = 0.01
  centre_leg_dist_x = 0.18
  front_leg_dist_x = 0.115
  front_left_dist_y = 0.137
  rear_leg_dist_x = 0.115
  rear_left_dist_y = -0.137
  x = np.array([ride_height,
               floor_length,
               centre_leg_dist_x,
               front_leg_dist_x,
               front_left_dist_y,
               rear_leg_dist_x,
               rear_left_dist_y])

  init_std_devs = np.array([0.01, 0.005, 0.01, 0.01, 0.01, 0.01, 0.01])
  try_limit = 400
  initial_cost = cost_fn(x, "initial_cost.ply")
  if initial_cost is None:
    print("## Error seed state for gait optimistion has no kinematic solution")
    return
  print("initial cost: %f" % initial_cost)

  solution_improved = True
  last_cost = initial_cost
  while solution_improved:
    solution_improved = False
    for att in range(try_limit):
      new_x = np.array(x)
      for idx, std_dev in enumerate(init_std_devs):
        new_x[idx] += np.random.normal(0, std_dev)
      new_cost = cost_fn(new_x)
      # print("att [%d] cost: %s" % (att, new_cost))
      if new_cost is not None and new_cost > last_cost:
        x = new_x
        last_cost = new_cost
        solution_improved = True
        break

    if solution_improved:
      print("Cost improved to: %f" % last_cost)

  cost_fn(x, "Final_gait_toes.ply")
  gait_trajectory = create_gait_trajectory(x, 0.5, "gait_debug.ply")

  print("Final trajectory\n---------------------------")
  idx = 0
  for row in gait_trajectory:
    line = "%d, " % idx
    for angle in row:
      line += "%f, " % angle
    idx += 1
    print(line)


if __name__ == "__main__":
  gen_walking_toe_trajectory(floor_distance=1.4, lift_height=1, floor_duration=0.5, save=True)
  generate_all_leg_index_test("leg_index_Test.ply")
  optimise_walking_gait()
