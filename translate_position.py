from math import sin, cos
import numpy as np
from scipy.spatial.transform import Rotation as SciRot
from leg_kinematics import LegKinematics

kin = LegKinematics()

LEG_FRONT_RIGHT = 0
LEG_MIDDLE_RIGHT = 1
LEG_REAR_RIGHT = 2
LEG_REAR_LEFT = 3
LEG_MIDDLE_LEFT = 4
LEG_FRONT_LEFT = 5

LEG_PIVOTS = {0:(0.0,-0.1,0.0), 1:(0.0,-0.1,0.0), 2:(0.0,-0.1,0.0), 3:(0.0,-0.1,0.0), 4:(0.0,-0.1,0.0), 5:(0.0,-0.1,0.0) } # in leg space, all legs are the same
LEG_ANGLES = {0:40.0, 1:90.0, 2:140.0, 3:-140.0, 4:-90.0, 5:-40.0}  # Angles in relation to +ve Y - Forward in robot space


def get_leg_base_frames():
    """
    returns a dictionary of 4x4 transformation matrices from the robot body frame to the base
    frames of all six legs
    :return: dict of 4x4 ndarrays
    """
    labels = ["front_right", "middle_right", "rear_right", "rear_left", "middle_left", "front_left"]
    frames = {}
    for idx, label in enumerate(labels):
        tf = np.eye(4)
        rot_vec = [0, 0, np.deg2rad(LEG_ANGLES[idx])]
        tf[0:3, 0:3] = SciRot.from_rotvec(rot_vec).as_matrix()
        tf[3, 0:3] = np.matmul((0.0, 0.1, 0.0), tf[0:3, 0:3])
        frames[label] = tf
    return frames


# Courtesy of ChatGPT
def translate_datum(point, leg):
    """
    Translate position in robot co-ordinate space to leg co-ordinate space
    Double rotation to as some rotationa are over 90 degrees
    """
    shift = np.array(LEG_PIVOTS[leg])
    theta = np.deg2rad(LEG_ANGLES[leg]) / 2.0

    # Compute the rotation matrix
    # z axis only
    R = np.array([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]])
    
    # Translate the point to the new datum
    new_point = np.matmul(R, point)
    new_point = np.matmul(R, new_point) + shift 
    return new_point


def inverse_translate_datum(point, leg):
    """
    Translate position in leg co-ordinate space to robot co-ordinate space
    Double rotation to as some rotationa are over 90 degrees
    """
    shift = np.array(LEG_PIVOTS[leg])
    theta = -np.deg2rad(LEG_ANGLES[leg])/2

    # Compute the rotation matrix
    # z axis only
    R = np.array([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]])
    # calculate the inverse matrix
    # R = np.linalg.inv(R)

    # Change origin
    point -= shift
    # Translate the point to the new datum
    new_point = np.matmul(R, point)
    new_point = np.matmul(R, new_point)
    return new_point


def joints_to_all_leg_positions(joint_angles):
    """
    Function to convert an array like object of 18 joint angles in radians to a dictionary of legs with
    the 3D position of all joints in the robot body frame
    :param joint_angles:
    :return: dictionary of 3D leg positions
    """
    positions = {}
    for leg_idx in range(6):
        leg_joint_angles = joint_angles[leg_idx*3:leg_idx*3+3]
        leg_positions = kin.forwards_all_joints(leg_joint_angles)
        for key in leg_positions.keys():
            leg_positions[key] = inverse_translate_datum(leg_positions[key], leg_idx)
        positions[leg_idx] = leg_positions

    return positions
