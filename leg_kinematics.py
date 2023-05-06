"""
Hexapod leg kinematics
----------------------

This module includes analytic forwards and inverse functions for a single leg, as well as an all joints forwards
kinematic function which returns the locations of all joints as well as the toe position.

 Coordinate frame definition
-----------------------------
                                Z
                   |            |
--====O=======O====|       Y----X
                   |

 Joint angle definitions
-------------------------

Leg zero pose, viewed from the side (up is up and down is down!):

      j3      j1   j0
 ^-ve    ^-ve      |
--====O=======O====|  (+ve j0 rotation moved the leg out of the screen)
 ,+ve    ,+ve      |

"""

import numpy as np


class NoKinematicSolution(Exception):
  pass


class LegKinematics:

  def __init__(self):

    # define link lengths
    self.hip_length = 26e-3
    self.thigh_length = 48.87e-3
    self.calf_length = 52.162e-3

    # define joint limits
    self.joint_low_limits = np.array([-np.pi/2, -np.pi/2, np.deg2rad(-60)])
    self.joint_high_limits = np.array([np.pi/2, np.pi/2, np.deg2rad(60)])

  def forwards(self, joint_angles):
    """
    Forwards kinematic function for a single leg. Converts a set of three
    joint angles into the 3D toe position in the base link frame
    :param joint_angles: 3 element array of angles in radians (range +/- pi)
    :return: toe position in a 3 element numpy.array in meters
    """
    joint_angles = np.array(joint_angles)
    assert joint_angles.shape == (3,), "joint_angles not a 3 element vector"

    # use the cosine rule to combine the two leg links into a single length and angle
    combined_leg_length = np.sqrt(
      self.calf_length**2 +
      self.thigh_length**2 -
      2 * self.calf_length * self.thigh_length * np.cos(np.pi - joint_angles[2])
    )
    combined_leg_angle = np.arccos(
      (self.thigh_length**2 + combined_leg_length**2 - self.calf_length**2) /
      (2 * self.thigh_length * combined_leg_length)
    ) * np.sign(joint_angles[2])

    # now use hip and thigh rotation to determine toe position.
    z = np.sin(joint_angles[1] + combined_leg_angle) * combined_leg_length
    y_pre_rotation = (np.cos(joint_angles[1] + combined_leg_angle) * combined_leg_length) + self.hip_length
    x = np.sin(joint_angles[0]) * y_pre_rotation
    y = np.cos(joint_angles[0]) * y_pre_rotation

    return np.array([x, y, z])

  def forwards_all_joints(self, joint_angles):
    """
    Forwards kinematic function for a single leg. Converts a set of three
    joint angles into the 3D position of all three joints and toe in the base link frame
    :param joint_angles: 3 element array of angles in radians (range +/- pi)
    :return: dictionary of labelled positions in 3 element numpy.arrays in meters
    """
    positions = {"joint_0": np.zeros((3,)),
                 "joint_1": np.array([self.hip_length, self.hip_length, 0])}

    knee_relative = np.array([0,
                              np.sin(joint_angles[1]) * self.thigh_length,
                              np.cos(joint_angles[1]) * self.thigh_length])
    positions["joint_2"] = positions["joint_1"] + knee_relative

    toe_relative = np.array([0,
                             np.sin(joint_angles[1] + joint_angles[2]) * self.calf_length,
                             np.cos(joint_angles[1] + joint_angles[2]) * self.calf_length])
    positions["toe"] = positions["joint_2"] + toe_relative

    for key in positions.keys():
      positions[key][0] = np.sin(joint_angles[0]) * positions[key][1]
      positions[key][1] = np.sin(joint_angles[0]) * positions[key][1]

    return positions

  def inverse(self, toe_position):
    """
    Inverse kinematic function for a single leg. Converts a toe position
    as a 3D vector in the base link frame to a set of joint angles. Throws
    NoKinematicSolution if no set of joint angles exist for the target position.
    Note this solver will always return the knee up solution!
    :param toe_position: 3 element vector in meters expressed in base link frame.
    :return: joint angles, 3 element vector of angles in radians
    """
    toe_x, toe_y, toe_z = toe_position

    # First compute the orthogonal edges the right angle triangle that the calf and thigh
    # links need to reach along the diagonal of
    leg_horizontal_reach = np.linalg.norm(toe_position[:2]) - self.hip_length
    leg_vertical_reach = toe_z
    leg_diagonal_reach = np.linalg.norm(np.array([leg_horizontal_reach, leg_vertical_reach]))

    # if the required leg reach is longer than thigh and calf links combined then there is no solution
    if leg_diagonal_reach > (self.calf_length + self.thigh_length):
      raise NoKinematicSolution("Diagonal leg Reach is too long.")

    # use the cosine rule to compute the knee angle and check it's valid
    knee_angle = np.pi - np.arccos(
      (self.calf_length**2 + self.thigh_length**2 - leg_diagonal_reach**2) /
      (2 * self.calf_length * self.thigh_length)
    )
    if knee_angle > self.joint_high_limits[2]:
      raise NoKinematicSolution("Knee angle greater than limit.")

    # use atan2 and cosine rule to compute thigh angle and check its in valid range
    tri_angle = np.arcsin(leg_horizontal_reach / leg_diagonal_reach)
    shoulder_partial_angle = np.arccos(
      (self.thigh_length**2 + leg_diagonal_reach**2 - self.calf_length**2) /
      (2 * self.thigh_length * leg_diagonal_reach)
    )
    shoulder_angle = shoulder_partial_angle + tri_angle
    if shoulder_angle < self.joint_low_limits[1] or shoulder_angle > self.joint_high_limits[1]:
      raise NoKinematicSolution("Shoulder angle is out of range.")

    # use atan2 to compute hip angle and check its in valid range
    hip_angle = np.arctan2(toe_x, toe_y)
    if hip_angle < self.joint_low_limits[0] or hip_angle > self.joint_high_limits[0]:
      raise NoKinematicSolution("Hip angle is out of range.")

    # return joint angle array
    return [hip_angle, shoulder_angle, knee_angle]
