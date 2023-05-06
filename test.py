import numpy as np
from leg_kinematics import LegKinematics as Kin
from ply_debug_file import PLYFile


leg_kin = Kin()


def add_joint_config_to_debug_file(file, joint_angles):
  leg_positions = leg_kin.forwards_all_joints(joint_angles)
  toe_pos = leg_kin.forwards(joint_angles)

  file.add_line(leg_positions["joint_0"], leg_positions["joint_1"], 255, 0, 0)
  file.add_line(leg_positions["joint_1"], leg_positions["joint_2"], 0, 255, 0)
  file.add_line(leg_positions["joint_2"], leg_positions["toe"], 0, 0, 255)
  file.add_line(leg_positions["joint_0"], toe_pos, 255, 255, 255)


def main():

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

  ply_debug = PLYFile()
  # compute joint trajectory sweeping the toe position in a circle
  radius = 0.01
  for angle in np.linspace(0, np.deg2rad(360), 100):
    goal_toe_pos = np.array([np.sin(angle) * radius,
                             (np.cos(angle) * radius) + 0.07,
                             -0.07])

    ply_debug.add_point(goal_toe_pos, 255, 255, 0)

    joint_angles = leg_kin.inverse(goal_toe_pos)
    leg_positions = leg_kin.forwards_all_joints(joint_angles)

    toe_pos = leg_kin.forwards(joint_angles)

    print("[%f] joint angles %s" % (angle, np.rad2deg(joint_angles)))
    print("joint_0 %s" % leg_positions["joint_0"])
    print("joint_1 %s" % leg_positions["joint_1"])
    print("joint_2 %s" % leg_positions["joint_2"])
    print("toe %s" % leg_positions["toe"])
    print("alt toe %s" % toe_pos)
    print("")

    add_joint_config_to_debug_file(ply_debug, joint_angles)

  ply_debug.save("leg_trajectory.ply")


if __name__ == "__main__":
  main()
