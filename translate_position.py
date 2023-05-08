from math import sin, cos
import numpy as np

LEG_FRONT_RIGHT = 0
LEG_MIDDLE_RIGHT = 1
LEG_REAR_RIGHT = 2
LEG_REAR_LEFT = 3
LEG_MIDDLE_LEFT = 4
LEG_FRONT_LEFT = 5

LEG_PIVOTS = {0:(0.0,-0.1,0.0), 1:(0.0,-0.1,0.0), 2:(0.0,-0.1,0.0), 3:(0.0,-0.1,0.0), 4:(0.0,-0.1,0.0), 5:(0.0,-0.1,0.0) } # in leg space, all legs are the same
LEG_ANGLES = {0:40.0, 1:90.0, 2:130.0, 3:-130.0, 4:-90.0, 5:-40.0}  # Angles in relation to +ve Y - Forward in robot space


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
