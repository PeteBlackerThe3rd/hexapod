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

Note: X axis points into the screen

 Joint angle definitions
-------------------------

Leg zero pose, viewed from the side (up is up and down is down!):

      j3      j1   j0
 ^-ve    ^-ve      |
--====O=======O====|  (+ve j0 rotation moves the leg out of the screen)
 ,+ve    ,+ve      |

"""
from scipy.spatial.transform import Rotation as SciRot
import numpy as np


class NoKinematicSolution(Exception):
  pass


class WorkingArea2D:
  """
  Class used to store and operate on the 2D horizontal working area for the foot
  of a single leg.
  This object can detect intersections of straight and curved foot trajectories with this
  boundary and generate a render line list of it
  """
  def __init__(self, inner_rad, outer_rad, pos_x_angle, neg_x_angle, foot_z):
    self.inner_rad = inner_rad
    self.outer_rad = outer_rad
    self.pos_x_angle = pos_x_angle
    self.neg_x_angle = neg_x_angle
    self.foot_z = foot_z

    self.pos_x_edge_pos = np.array(np.sin(np.deg2rad(90) - pos_x_angle), np.cos(np.deg2rad(90) - pos_x_angle))
    self.neg_x_edge_pos = np.array(np.sin(np.deg2rad(90) - neg_x_angle), np.cos(np.deg2rad(90) - neg_x_angle))

  def get_boundary_points(self):
    """
    Generates a list of 3D point describing the boundary of thie working area
    :return: Nx3 2D numpy ndarray
    """
    point_count = 60
    points = np.zeros((point_count*2, 3))
    for idx, angle in enumerate(np.linspace(self.pos_x_angle, self.neg_x_angle, point_count)):
      rot_angle = angle
      points[idx, :] = [np.sin(rot_angle) * self.inner_rad, np.cos(rot_angle) * self.inner_rad, self.foot_z]
    for idx, angle in enumerate(np.linspace(self.neg_x_angle, self.pos_x_angle, point_count)):
      rot_angle = angle
      points[idx + point_count, :] = [np.sin(rot_angle) * self.outer_rad,
                                      np.cos(rot_angle) * self.outer_rad,
                                      self.foot_z]
    return points


class LegKinematics:
  """
  Kinematic model of a single standard hexapod leg. Can be customised to the specific
  geometry of a particular leg, but the joint type order is fixed.
  """

  def __init__(self):

    # define link lengths in meters
    self.hip_length = 26e-3
    self.thigh_length = 48.87e-3
    self.calf_length = 52.162e-3 + 2.0e-3 # Rubber boot adds about 2mm

    # define joint limits
    self.joint_low_limits = np.array([np.deg2rad(-20), np.deg2rad(-80), np.deg2rad(0)])
    self.joint_high_limits = np.array([np.deg2rad(20), np.deg2rad(80), np.deg2rad(120)])

  def compute_vector_working_area(self, foot_height):
    """
    Computes the two radii and two line with bound the singularity free working volume of
    this leg.
    :param foot_height: float, the height of the foot in the Z coordinate, so negative if below the hip
    :return: A 2D working area object
    """
    min_hip_singularity_dist = 0.02  # limit of 2 cm from singularity
    min_knee_angle = np.deg2rad(10)  # limit of 10 degrees from 'straight-leg' singularity

    # solve for the maximum leg reach using trig
    # use the cosine rule to combine the two leg links into a single length and angle
    combined_leg_length = np.sqrt(
      self.calf_length**2 +
      self.thigh_length**2 -
      2 * self.calf_length * self.thigh_length * np.cos(np.pi - min_knee_angle)
    )
    # use pythagoras to find the maximum leg reach
    combined_leg_reach = np.sqrt(combined_leg_length**2 - foot_height**2)
    max_leg_reach = combined_leg_reach + self.hip_length

    # Solving for the minimum leg reach is a bit more complicated because it can be limited by three different
    # things. The knee joint limit, the thigh joint limit, or the singularity distance
    min_leg_reach = 0.01  # TODO

    return WorkingArea2D(max(min_leg_reach, min_hip_singularity_dist),
                         max_leg_reach,
                         self.joint_high_limits[0],
                         self.joint_low_limits[0],
                         foot_height)

  def compute_joint_limit_margin(self, joint_angles):
    """
    Computes the smallest angle between any of the joint angles given and their
    respective limits. Used as an approximate metric of the distance to the
    edge of the working volume.
    :param joint_angles: np.ndarray of three joint angles
    :return: scalar of the margin in radians
    """
    return min(np.min(joint_angles - self.joint_low_limits), np.min(self.joint_high_limits - joint_angles))

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
    z = -np.sin(joint_angles[1] + combined_leg_angle) * combined_leg_length
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
                 "joint_1": np.array([0, self.hip_length, 0])}

    knee_relative = np.array([0,
                              np.cos(joint_angles[1]) * self.thigh_length,
                              -np.sin(joint_angles[1]) * self.thigh_length])
    positions["joint_2"] = positions["joint_1"] + knee_relative

    toe_relative = np.array([0,
                             np.cos(joint_angles[1] + joint_angles[2]) * self.calf_length,
                             -np.sin(joint_angles[1] + joint_angles[2]) * self.calf_length])
    positions["toe"] = positions["joint_2"] + toe_relative

    for key in positions.keys():
      positions[key][0] = np.sin(joint_angles[0]) * positions[key][1]
      positions[key][1] = np.cos(joint_angles[0]) * positions[key][1]

    return positions

  def forwards_all_frames(self, joint_angles):
    """
    Forwards kinematic function for a single leg. Converts a set of three
    joint angles into the 4x4 transformation matrices of all three links
    :param joint_angles: 3 element array of angles in radians (range +/- pi)
    :return: dictionary of labelled 4x4 ndarray transformation matrices
    """
    # init all frames to identity
    frames = {"link_0": np.eye(4), "link_1": np.eye(4), "link_2": np.eye(4)}

    # link zero is just a rotation about the leg origin
    frames["link_0"][0:3, 0:3] = SciRot.from_rotvec([0, 0, joint_angles[0]]).as_matrix()

    # link one is a translation by hip length and rotation
    frames["link_1"][0:3, 0:3] = SciRot.from_rotvec([joint_angles[1], 0, 0]).as_matrix()
    frames["link_1"][3, 1] = self.hip_length
    frames["link_1"] = np.matmul(frames["link_1"], frames["link_0"])

    # link two is translation by thigh length and rotation
    frames["link_2"][0:3, 0:3] = SciRot.from_rotvec([joint_angles[2], 0, 0]).as_matrix()
    frames["link_2"][3, 1] = self.thigh_length
    frames["link_2"] = np.matmul(frames["link_2"], frames["link_1"])

    return frames

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
    leg_horizontal_reach = np.linalg.norm(-toe_position[:2]) - self.hip_length
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
      raise NoKinematicSolution("Knee angle (%f deg) greater than high limit (%f def)." % (
        np.rad2deg(knee_angle),
        np.rad2deg(self.joint_high_limits[2])
      ))

    # use atan2 and cosine rule to compute thigh angle and check its in valid range
    tri_angle = np.arctan2(-toe_z, leg_horizontal_reach)
    shoulder_partial_angle = np.arccos(
      (self.thigh_length**2 + leg_diagonal_reach**2 - self.calf_length**2) /
      (2 * self.thigh_length * leg_diagonal_reach)
    )
    shoulder_angle = tri_angle - shoulder_partial_angle
    if shoulder_angle < self.joint_low_limits[1] or shoulder_angle > self.joint_high_limits[1]:
      raise NoKinematicSolution("Shoulder angle is out of range.")

    # use atan2 to compute hip angle and check its in valid range
    hip_angle = np.arctan2(toe_x, toe_y)
    if hip_angle < self.joint_low_limits[0] or hip_angle > self.joint_high_limits[0]:
      raise NoKinematicSolution("Hip angle is out of range.")

    # return joint angle array
    return [hip_angle, shoulder_angle, knee_angle]
