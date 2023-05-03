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
    self.joint_low_limits = np.array([-np.pi/2, -np.pi/2, -np.pi/2])
    self.joint_high_limits = np.array([np.pi/2, np.pi/2, np.pi/2])

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
    combined_leg_angle = np.acos(
      (self.thigh_length**2 + combined_leg_length**2 - self.calf_length**2) /
      (2 * self.thigh_length * combined_leg_length)
    ) * np.sign(joint_angles[2])

    # now use hip and thigh rotation to determine toe position.
    z = np.sin(joint_angles[1] + combined_leg_angle) * combined_leg_length
    y_pre_rotation = (np.cos(joint_angles[1] + combined_leg_angle) * combined_leg_length) + self.hip_length
    x = np.sin(joint_angles[0]) * y_pre_rotation
    y = np.cos(joint_angles[0]) * y_pre_rotation

    return np.array([x, y, z])

  def inverse(self, toe_position):
    """
    Inverse kinematic function for a single leg. Converts a toe position
    as a 3D vector in the base link frame to a set of joint angles. Throws
    NoKinematicSolution if no set of joint angles exist for the target position.
    Note this solver will always return the knee up solution!
    :param toe_position: 3 element vector in meters expressed in base link frame.
    :return: joint angles, 3 element vector of angles in radians
    """

    # First compute the orthogonal edges the right angle triangle that the calf and thigh
    # links need to reach along the diagonal of
    leg_horizontal_reach = np.linalg.norm(toe_position[:2]) - self.hip_length
    leg_vertical_reach = toe_position[2]
    leg_diagonal_reach = np.linalg.norm(np.array([leg_horizontal_reach, leg_vertical_reach]))

    # if the required leg reach is longer than thigh and calf links combined then there is no solution
    if leg_diagonal_reach > (self.calf_length + self.thigh_length):
      raise NoKinematicSolution("Diagonal leg Reach is too long.")

    # use the cosine rule to compute the knee angle and check it's valid
    knee_angle = np.pi - np.acos(
      (self.calf_length**2 + self.thigh_length**2 - leg_diagonal_reach**2) /
      (2 * self.calf_length * self.thigh_length)
    )
    if knee_angle > self.joint_high_limits[2]:
      raise NoKinematicSolution("Knee angle greater than limit")

    # TODO use atan2 and cosine rule to compute thigh angle and check its in valid range

    # TODO use atan2 to compute hip angle and check its in valid range

    # return joint angle array
