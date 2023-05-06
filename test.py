import numpy as np
from leg_kinematics import LegKinematics as Kin, NoKinematicSolution
from ply_debug_file import PLYFile


leg_kin = Kin()


def add_joint_config_to_debug_file(file, joint_angles):
  leg_positions = leg_kin.forwards_all_joints(joint_angles)
  # toe_pos = leg_kin.forwards(joint_angles)

  file.add_line(leg_positions["joint_0"], leg_positions["joint_1"], 255, 0, 0)
  file.add_line(leg_positions["joint_1"], leg_positions["joint_2"], 0, 255, 0)
  file.add_line(leg_positions["joint_2"], leg_positions["toe"], 0, 0, 255)
  # file.add_line(leg_positions["joint_0"], toe_pos, 255, 255, 255)


def test_forwards_kinematics():
  ply_debug = PLYFile()
  for angle in np.linspace(np.deg2rad(-90), np.deg2rad(90), 100):
    joint_angles = [0, 0, angle]
    add_joint_config_to_debug_file(ply_debug, joint_angles)
  ply_debug.save("knee_joint_test.ply")

  ply_debug = PLYFile()
  for angle in np.linspace(np.deg2rad(-90), np.deg2rad(90), 100):
    joint_angles = [0, angle, 0]
    add_joint_config_to_debug_file(ply_debug, joint_angles)
  ply_debug.save("shoulder_joint_test.ply")

  ply_debug = PLYFile()
  for angle in np.linspace(np.deg2rad(-30), np.deg2rad(30), 100):
    joint_angles = [angle, 0, 0]
    add_joint_config_to_debug_file(ply_debug, joint_angles)
  ply_debug.save("hip_joint_test.ply")


def test_inverse_kinematics():

  zero_joint_toe_pos = np.array([0, leg_kin.hip_length + leg_kin.thigh_length + leg_kin.calf_length, 0])
  joint_angles = leg_kin.inverse(zero_joint_toe_pos)
  print("zero joint toe pos: joint angles: %s" % np.rad2deg(joint_angles))


def save_toe_working_area_plot(filename, hip_heights):

  plot = PLYFile()
  valid_count = 0

  for hip_height in hip_heights:
    for x in np.linspace(-0.05, 0.05, 100):
      for y in np.linspace(0.01, 0.15, 200):
        goal = np.array([x, y, hip_height])
        try:
          joint_angles = leg_kin.inverse(goal)
          margin = leg_kin.compute_joint_limit_margin(joint_angles)
          valid_count += 1
          max_margin = np.deg2rad(30)
          margin_ratio = margin / max_margin
          r, g, b = [int((1 - margin_ratio) * 255), int(margin_ratio * 255), 0]
          plot.add_point(goal, r, g, b)
        except NoKinematicSolution:
          pass

  plot.save(filename)


def main():

  # save_toe_working_area_plot("test_plot.ply", np.linspace(-0.10, -0.02, 12))

  ply_debug = PLYFile()
  # compute joint trajectory sweeping the toe position in a circle
  radius = 0.018
  print("index, joint_0 rads, join_1 rads, join_2 rads")
  for idx, angle in enumerate(np.linspace(0, np.deg2rad(360), 200)):
    goal_toe_pos = np.array([np.sin(angle) * radius,
                             (np.cos(angle) * radius) + 0.08,
                             -0.07])

    joint_angles = leg_kin.inverse(goal_toe_pos)
    print("%d, %f, %f, %f" % (idx, joint_angles[0], joint_angles[1], joint_angles[2]))

    # leg_positions = leg_kin.forwards_all_joints(joint_angles)
    # print("[%f] joint angles %s" % (angle, np.rad2deg(joint_angles)))
    # print("joint_0 %s" % leg_positions["joint_0"])
    # print("joint_1 %s" % leg_positions["joint_1"])
    # print("joint_2 %s" % leg_positions["joint_2"])
    # print("toe %s" % leg_positions["toe"])
    # print("")

    add_joint_config_to_debug_file(ply_debug, joint_angles)

  ply_debug.save("leg_trajectory.ply")


if __name__ == "__main__":
  main()
