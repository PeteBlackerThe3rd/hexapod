import numpy as np
import math
from leg_kinematics import LegKinematics as Kin, NoKinematicSolution
from ply_debug_file import PLYFile
from translate_position import translate_datum, inverse_translate_datum
from time import sleep
from robot import robot
import struct

leg_kin = Kin()

hexy = robot()

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

def circle():
  radius = 0.010
  for idx, angle in enumerate(np.linspace(0, np.deg2rad(360), 720)):
    goal_toe_pos = np.array([np.sin(angle) * radius,
                             (np.cos(angle) * radius) + 0.090,
                             -0.065])
    joint_angles = leg_kin.inverse(goal_toe_pos)
    yield idx, joint_angles

def circle_translated():
  radius = 0.01
  for idx, angle in enumerate(np.linspace(0, np.deg2rad(360), 720)):
    goal_toe_pos = np.array([np.sin(angle) * radius+0.11, 0.18,
                             (np.cos(angle) * radius)])
    goal_toe_pos = translate_datum(goal_toe_pos, 0)
    joint_angles = leg_kin.inverse(goal_toe_pos)
    yield idx, joint_angles

def linear_translated():
  for idx, pos in enumerate(np.linspace(-0.02, 0.02, 100)):
    goal_toe_pos = np.array([0.10, pos + 0.12, -0.086])
    goal_toe_pos = translate_datum(goal_toe_pos, 0)
    joint_angles = leg_kin.inverse(goal_toe_pos)
    yield idx, joint_angles

def initialise_robot():
  """
  Sets all hip joint positions to zero and all knee and ankle joints to 45 degrees
  Code is more a test of the funtions rather than useful
  """
  joint_angles = np.array([0,math.pi/4, math.pi/4])
  leg_neutral_toe_position = leg_kin.forwards((0,math.pi/4, math.pi/4))
  for leg in range(0,6):
    print("------------------------------------------------------------")
    leg_neutral_toe_position = leg_kin.forwards((0,math.pi/4, math.pi/4))
    #joint_angles = leg_kin.inverse(leg_neutral_toe_position)
    print(leg_neutral_toe_position)
    hexy.set_leg_joint_angles(joint_angles, leg)
  hexy.send()

def main():
  #save_toe_working_area_plot("test_plot.ply", np.linspace(-0.10, -0.02, 12))
  ply_debug = PLYFile()

  initialise_robot()
  
  steps = 100
  z = -0.05
  for _step in range(steps):
    hexy.move_body(np.array([0.0,0,z/steps]))
  sleep(2)
  z = 0.03
  for _step in range(steps):
    hexy.move_body(np.array([0.0,0,z/steps]))
  
  print("Starting Trajectory")
  hexy.follow_trajectory("walking_trajectory.csv", cycles=10, speed=2)


if __name__ == "__main__":
  main()
