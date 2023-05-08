#Message Structure

#  B<<18 unsigned bytes>> set servo positions
#  T<<18 signed bytes>> set servo trim positions

from time import sleep, monotonic_ns
import serial
import numpy as np
from math import pi
import struct
from leg_kinematics import LegKinematics as Kin, NoKinematicSolution
from translate_position import translate_datum, inverse_translate_datum

robot_leg_kin = Kin()

# Device specific information
# Serial
PORT = 'COM3'

TRIM = [0]*18
TRIM[0] = -16
TRIM[1] = -11
TRIM[2] =  -10
TRIM[3] = 21
TRIM[4] = -4
TRIM[5] = 2
TRIM[6] = -9
TRIM[7] = 5
TRIM[8] = -9
TRIM[9] = -16
TRIM[10] = 5
TRIM[11] = -10
TRIM[12] = 7
TRIM[13] = -2
TRIM[14] = 4
TRIM[15] = 10
TRIM[16] = -7
TRIM[17] = 4

SERVO_DIR = [1]*18
SERVO_DIR[0] = 1
SERVO_DIR[1] = 1
SERVO_DIR[2] = -1
SERVO_DIR[3] = 1
SERVO_DIR[4] = 1
SERVO_DIR[5] = -1
SERVO_DIR[6] = 1
SERVO_DIR[7] = 1
SERVO_DIR[8] = -1
SERVO_DIR[9] = 1
SERVO_DIR[10] = 1
SERVO_DIR[11] = -1
SERVO_DIR[12] = 1
SERVO_DIR[13] = 1
SERVO_DIR[14] = -1
SERVO_DIR[15] = 1
SERVO_DIR[16] = 1
SERVO_DIR[17] = -1


def calculate_servo_positions(angles):
    positions = [150]*18
    for i, angle in enumerate(angles):
        pos = 150 + (angle/(pi/2)) * 100 * SERVO_DIR[i]
        if pos < 50:
            pos = 50
        elif pos > 250:
            pos = 250
        positions[i] = int(pos)
    return positions

def calculate_servo_position(angle, servo):
    pos = 150 + (angle/(pi/2) * 100 * SERVO_DIR[servo])
    if pos < 50:
        pos = 50
    elif pos > 250:
        pos = 250
    return int(pos)

class robot:
    """
    Hexapod robot class 
    """
    def __init__(self):
        # Connect to Serial
        self.s = serial.Serial(PORT, baudrate=115200, timeout=1)
        recv = ""
        while 'SERVO' not in recv:
            sleep(1)
            self.s.write(b'V\n')
            recv = self.s.readline().decode("utf-8")
            print("connecting", recv)
        print('connected')

        # Trim Servos
        self.s.write(b'T')
        for tval in TRIM:
            self.s.write(struct.pack('b', tval))
        self.last_send_time = monotonic_ns()
        self.servo_pos = [150]*18
        self.absolute_toe_positions = [np.array([0,0,0])]*6

    def set_leg_joint_angles(self, joint_angles, leg):
        """
        joint angles are in radians
        """
        self.servo_pos[leg*3] = calculate_servo_position(joint_angles[0], leg*3)
        self.servo_pos[leg*3 + 1] = calculate_servo_position(joint_angles[1], leg*3 + 1)
        self.servo_pos[leg*3 + 2] = calculate_servo_position(joint_angles[2], leg*3 + 2)
        toe_pos = robot_leg_kin.forwards(np.array(joint_angles))
        self.absolute_toe_positions[leg] = inverse_translate_datum(toe_pos, leg)

    def set_toe_position_relative(self, goal_toe_position, leg):
        """
        toe_position in leg co-ordinate space
        """
        joint_angles = robot_leg_kin.inverse(goal_toe_position)
        self.servo_pos[leg*3] = calculate_servo_position(joint_angles[0], leg*3)
        self.servo_pos[leg*3 + 1] = calculate_servo_position(joint_angles[1], leg*3 + 1)
        self.servo_pos[leg*3 + 2] = calculate_servo_position(joint_angles[2], leg*3 + 2)
        toe_position = inverse_translate_datum(goal_toe_position, leg)
        self.absolute_toe_positions[leg] = toe_position
        pass

    def set_toe_position_absolute(self, toe_position, leg):
        """
        toe_position in robot co-ordinate space
        """
        goal_toe_position = translate_datum(toe_position, leg)
        joint_angles = robot_leg_kin.inverse(goal_toe_position)
        self.servo_pos[leg*3] = calculate_servo_position(joint_angles[0], leg*3)
        self.servo_pos[leg*3 + 1] = calculate_servo_position(joint_angles[1], leg*3 + 1)
        self.servo_pos[leg*3 + 2] = calculate_servo_position(joint_angles[2], leg*3 + 2)
        self.absolute_toe_positions[leg] = toe_position

    def move_body(self, movement_vector):
        """
        Move robot body in direction of vector
        """
        leg_vector = movement_vector * -1
        for idx, position in enumerate(self.absolute_toe_positions):
            self.set_toe_position_absolute(position + leg_vector, idx)
        self.send()


    def send(self):
        # limit writes to 50Hz
        sleep(max([0.02 - (monotonic_ns() - self.last_send_time)/1e9,0]))
        self.s.write(b'B'+bytes(self.servo_pos))
        self.last_send_time = monotonic_ns()



                

