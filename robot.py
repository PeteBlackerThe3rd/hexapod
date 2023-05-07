#Message Structure

#  B<<18 unsigned bytes>> set servo positions
#  T<<18 signed bytes>> set servo trim positions

from time import sleep, monotonic_ns
import serial
import numpy as np
from math import pi
import struct

# Device specific information
# Serial
PORT = 'COM3'

trim = [0]*18
trim[0] = -16
trim[1] = -11
trim[2] =  -10
trim[3] = 21
trim[4] = -4
trim[5] = 2
trim[6] = -9
trim[7] = 5
trim[8] = -9
trim[9] = -16
trim[10] = 5
trim[11] = -10
trim[12] = 7
trim[13] = -2
trim[14] = 4
trim[15] = 10
trim[16] = -7
trim[17] = 4

servo_dir = [1]*18
servo_dir[0] = 1
servo_dir[1] = 1
servo_dir[2] = -1
servo_dir[3] = 1
servo_dir[4] = 1
servo_dir[5] = -1
servo_dir[6] = 1
servo_dir[7] = 1
servo_dir[8] = -1
servo_dir[9] = 1
servo_dir[10] = 1
servo_dir[11] = -1
servo_dir[12] = 1
servo_dir[13] = 1
servo_dir[14] = -1
servo_dir[15] = 1
servo_dir[16] = 1
servo_dir[17] = -1


def calculate_servo_positions(angles):
    positions = [150]*18
    for i, angle in enumerate(angles):
        pos = 150 + (angle/(pi/2)) * 100 * servo_dir[i]
        if pos < 50:
            pos = 50
        elif pos > 250:
            pos = 250
        positions[i] = int(pos)
    return positions

def calculate_servo_position(angle, servo):
    pos = 150 + (angle/(pi/2) * 100 * servo_dir[servo])
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
        for tval in trim:
            self.s.write(struct.pack('b', tval))
        self.last_send_time = monotonic_ns()
        self.servo_pos = [150]*18

    def set_leg(self, joint_angles, leg):
        """
        joint angles are in radians
        """
        self.servo_pos[leg*3] = calculate_servo_position(joint_angles[0], leg*3)
        self.servo_pos[leg*3 + 1] = calculate_servo_position(joint_angles[1], leg*3 + 1)
        self.servo_pos[leg*3 + 2] = calculate_servo_position(joint_angles[2], leg*3 + 2)

    def send(self):
        # limit writes to 50Hz
        sleep(max([0.02 - (monotonic_ns() - self.last_send_time)/1e9,0]))
        self.s.write(b'B'+bytes(self.servo_pos))
        self.last_send_time = monotonic_ns()



                

