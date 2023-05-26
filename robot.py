#Message Structure

#  B<<18 unsigned bytes>> set servo positions
#  T<<18 signed bytes>> set servo trim positions

from time import sleep, monotonic_ns
import serial
import numpy as np
from math import pi
import struct
from copy import copy
from leg_kinematics import LegKinematics as Kin, NoKinematicSolution
from translate_position import translate_datum, inverse_translate_datum

from array import array

robot_leg_kin = Kin()

FREQUENCY = 125
MIN_PULSE_LENGTH = 550
MAX_PULSE_LENGTH = 2100
CENTRE_PULSE_LENGTH = int((MAX_PULSE_LENGTH - MIN_PULSE_LENGTH) /2 + MIN_PULSE_LENGTH)
PULSE_DELTA = MAX_PULSE_LENGTH - MIN_PULSE_LENGTH

# Device specific information
# Serial
PORT = 'COM12'

TRIM = [0]*18
TRIM[0] = -160
TRIM[1] = -110
TRIM[2] = -300
TRIM[3] = 210
TRIM[4] = -40
TRIM[5] = 20
TRIM[6] = -90
TRIM[7] = 50
TRIM[8] = -90
TRIM[9] = -160
TRIM[10] = 50
TRIM[11] = -100
TRIM[12] = 70
TRIM[13] = -20
TRIM[14] = 40
TRIM[15] = 100
TRIM[16] = -70
TRIM[17] = 40

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

ANKLE_SERVOS = [2, 5, 8, 11, 14, 17]


def calculate_servo_positions(angles):
    positions = [1500]*18
    for i, angle in enumerate(angles):
        if i in ANKLE_SERVOS:
            # Ankle servos are offset by 90 degrees
            angle -= pi/2
        pos = CENTRE_PULSE_LENGTH + (angle/(pi/2)) * PULSE_DELTA * SERVO_DIR[i]
        if pos < MIN_PULSE_LENGTH:
            pos = MIN_PULSE_LENGTH
        elif pos > MAX_PULSE_LENGTH:
            pos = MAX_PULSE_LENGTH
        positions[i] = int(pos)
    return positions


def calculate_servo_position(angle, servo):
    if servo in ANKLE_SERVOS:
        # Ankle servos are offset by 90 degrees
        angle -= pi/2
    pos = CENTRE_PULSE_LENGTH + (angle/(pi/2)) * PULSE_DELTA * SERVO_DIR[servo]
    if pos < MIN_PULSE_LENGTH:
        pos = MIN_PULSE_LENGTH
    elif pos > MAX_PULSE_LENGTH:
        pos = MAX_PULSE_LENGTH
    return int(pos)


class robot:
    """
    Hexapod robot class 
    """
    def __init__(self):
        # Connect to Serial
        self.s = serial.Serial(PORT, baudrate=115200, timeout=0.5)
        recv = ""
        while 'rp2040' not in recv:
            sleep(5)
            self.s.write(b'V\n')
            recv = self.s.readline().decode("utf-8")
            print("connecting", recv)
        print('connected')

        # Trim Servos
        self.s.write(b'T')
        for tval in TRIM:
          # Send as int16_t little endian
          self.s.write(struct.pack('<h', tval))
        self.last_send_time = monotonic_ns()
        self.servo_pos = [CENTRE_PULSE_LENGTH]*18
        self.absolute_toe_positions = [np.array([0, 0, 0])]*6

    def set_leg_joint_angles(self, joint_angles, leg):
        """
        joint angles are in radians
        """
        self.servo_pos[leg*3] = calculate_servo_position(joint_angles[0], leg*3)
        self.servo_pos[leg*3 + 1] = calculate_servo_position(joint_angles[1], leg*3 + 1)
        self.servo_pos[leg*3 + 2] = calculate_servo_position(joint_angles[2], leg*3 + 2)
        toe_pos = robot_leg_kin.forwards(np.array(joint_angles))
        self.absolute_toe_positions[leg] = inverse_translate_datum(toe_pos, leg)

    def set_joint_angles(self, joint_angles, leg):
        """
        joint angles are in radians
        np.array of all 18 joint angles
        """
        for idx, angle in enumerate(joint_angles):
            self.servo_pos[idx] = calculate_servo_position(joint_angles[0])

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

    def follow_trajectory(self, filename, cycles=1, speed=1):
        """
        trajectory is an array of 18 joint positions in radians
        """
        trajectory_steps = []
        with open(filename) as joint_angle_file:
            for line in joint_angle_file:
                joint_angle_text = line.split(',')[1:]
                # Skips errors in file
                if len(joint_angle_text) < 18:
                    print("ERROR")
                    continue
                for leg in range(0, 6):
                    joint_angles = np.array([float(joint_angle_text[leg*3]), float(joint_angle_text[leg*3+1]), float(joint_angle_text[leg*3+2])])
                    self.set_leg_joint_angles(joint_angles, leg)
                trajectory_steps.append(copy(self.servo_pos))
        skip_count = 0
        for _ in range(cycles):
            for joint_pos in trajectory_steps:
                skip_count += 1
                if skip_count < speed:
                    continue
                skip_count = 0
                self.servo_pos = joint_pos
                # print(cycle, joint_pos)
                self.send()

    def send(self):
        # limit writes to FREQUENCY Hz
        sleep(max([(1/FREQUENCY) - (monotonic_ns() - self.last_send_time)/1e9,0]))
        self.s.write(b"B")
        # Send as uint16_t little endian
        self.s.write(array('H', self.servo_pos).tobytes())
        self.last_send_time = monotonic_ns()

    def send_joint_angles(self, joint_angles):
        assert len(joint_angles) == 18, "send_joint_angles fails, joint_angles param doesn't contain 18 values";
        for idx, joint_angle in enumerate(joint_angles):
            self.servo_pos[idx] = calculate_servo_position(joint_angle, idx)
        self.send()
                

