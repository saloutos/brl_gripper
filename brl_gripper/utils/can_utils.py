# defining constants and helper functions for CANFD messages to hand and sensors

# imports
import numpy as np
from enum import Enum

# motor limits
P_MIN = -12.5
P_MAX = 12.5
V_MIN = -300.0
V_MAX = 300.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 10.0
T_MIN = -72.0
T_MAX = 72.0

FT_MIN = -20.0
FT_MAX = 20.0
FN_MIN = -30.0
FN_MAX = 30.0
ANG_MIN = -135.0
ANG_MAX =45.0

SCALE = 50.0

CAN_MSG_NUM = 2

MM_MAX = 14.0
MM_MIN = -14.0

# CAN IDs
GRIPPER_ENABLE_ID =     1
MOTOR_DATA =            2
SENSOR_DATA =           3
LEFT_COMMAND =          4
RIGHT_COMMAND =         5
WRIST_ID =              5

# Gripper hardware modes
class HandControlMode(Enum):
	CURRENT_CONTROL = 0
	POSITION_CONTROL = 1
	SENSOR_DEBUG_MODE = 2
	DISABLE_CONTROL = 3

# CAN messages
HAND_MODE_MSGS = {
HandControlMode.CURRENT_CONTROL : [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC] + ([0]*40),
HandControlMode.POSITION_CONTROL : [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD] + ([0]*40),
HandControlMode.SENSOR_DEBUG_MODE : [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB] + ([0]*40),
HandControlMode.DISABLE_CONTROL : [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFA] + ([0]*40) }

# U6 modes (wrist roll motor)
U6_Zero = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
U6_EnterMotorMode = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
U6_ExitMotorMode = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]

# helper functions
def float_to_uint(x, x_min, x_max, bits):
	span = x_max - x_min
	offset = x_min
	x_clip = max( min( x, x_max), x_min)
	return int(((x_clip-offset)*float((1 << bits)-1))/span)

def uint_to_float(x_int, x_min, x_max, bits):
	span = x_max - x_min
	offset = x_min
	x_map = float(x_int)*span/(float((1<<bits) - 1)) + offset
	return max( min( x_map, x_max), x_min)

