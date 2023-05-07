from math import sin, cos
import numpy as np

LEG_PIVOTS = (0.0,-0.10,0.0) # in leg space, all legs are the same
LEG_ANGLES = {0:40, 1:90, 2:130, 3:-130, 4:-90, 5:-40} # Angles in relation to +ve Y - Forward in robot space

# Courtesy of ChatGPT
def translate_datum(point, leg):
    shift = np.array(LEG_PIVOTS)
    theta = np.deg2rad(LEG_ANGLES[leg])

    # Compute the rotation matrix
    # z axis only
    R = np.array([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]])

    # Translate the point to the new datum
    new_point = np.matmul(R, point) + shift

    return new_point
