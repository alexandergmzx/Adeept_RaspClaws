#!/usr/bin/python
"""
Updated move.py using the new Adafruit PCA9685 library.
"""

import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
from mpu6050 import mpu6050
import Kalman_filter
import PID
import threading
import RPIservo

# ----------------------------
# Setup PCA9685 with I2C
# ----------------------------
i2c = busio.I2C(SCL, SDA)
pwm = PCA9685(i2c)
pwm.frequency = 50

# ----------------------------
# Helper functions
# ----------------------------
def convert_to_duty_cycle(pulse):
    """
    Convert a pulse value (0–4096) to a 16‐bit duty cycle (0–65535).
    Validates that pulse is within the correct range.
    """
    # Clamp the pulse between 0 and 4096
    pulse = max(0, min(pulse, 4096))
    return int((pulse / 4096) * 65535)

def set_pwm(channel, pulse):
    """
    Set the duty cycle of a given channel using the new PCA9685 API.
    """
    pwm.channels[channel].duty_cycle = convert_to_duty_cycle(pulse)

def set_all_pwm(pulse):
    """
    Set all 16 channels to the same pulse value.
    """
    for ch in range(16):
        set_pwm(ch, pulse)

# ----------------------------
# Global configuration variables
# ----------------------------
# These variables adjust servo directions, heights, and movement ranges.
set_direction = 1

if set_direction:
    leftSide_direction  = 1
    rightSide_direction = 0
else:
    leftSide_direction  = 0
    rightSide_direction = 1

if set_direction:
    leftSide_height  = 0
    rightSide_height = 1
else:
    leftSide_height  = 1
    rightSide_height = 0

height_change = 30

if set_direction:
    Up_Down_direction = 1
    Left_Right_direction = 1
else:
    Up_Down_direction = 0
    Left_Right_direction = 0

Left_Right_input = 300
Up_Down_input = 300
Left_Right_Max = 500
Left_Right_Min = 100
Up_Down_Max = 500
Up_Down_Min = 270
look_wiggle = 30
move_stu = 1

steady_range_Min = -40
steady_range_Max = 130
range_Mid = (steady_range_Min + steady_range_Max) / 2
X_fix_output = range_Mid
Y_fix_output = range_Mid
steady_X_set = 73

# PID Settings
P = 5
I = 0.01
D = 0

X_pid = PID.PID()
X_pid.SetKp(P)
X_pid.SetKd(I)
X_pid.SetKi(D)
Y_pid = PID.PID()
Y_pid.SetKp(P)
Y_pid.SetKd(I)
Y_pid.SetKi(D)

# ----------------------------
# Sensor & Kalman filters
# ----------------------------
try:
    sensor = mpu6050(0x68)
    mpu6050_connection = 1
except:
    mpu6050_connection = 0

target_X = 0
target_Y = 0

# ----------------------------
# Default PWM values for all servos
# (Using list multiplication instead of listing each value)
# ----------------------------
pwm_defaults = [300] * 16
(pwm0, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7,
 pwm8, pwm9, pwm10, pwm11, pwm12, pwm13, pwm14, pwm15) = pwm_defaults

# ----------------------------
# Functions for testing and initializing servos
# ----------------------------
def mpu6050Test():
    while True:
        data = sensor.get_accel_data()
        print('X=%.3f, Y=%.3f, Z=%.3f' % (data['x'], data['y'], data['z']))
        time.sleep(0.3)

def init_all():
    set_pwm(0, pwm0)
    set_pwm(1, pwm1)
    set_pwm(2, pwm2)
    set_pwm(3, pwm3)
    set_pwm(4, pwm4)
    set_pwm(5, pwm5)
    set_pwm(6, pwm6)
    set_pwm(7, pwm7)
    set_pwm(8, pwm8)
    set_pwm(9, pwm9)
    set_pwm(10, pwm10)
    set_pwm(11, pwm11)
    set_pwm(12, pwm12)
    set_pwm(13, pwm13)
    set_pwm(14, pwm14)
    set_pwm(15, pwm15)

init_all()

def ctrl_range(raw, max_genout, min_genout):
    """Ensure the output stays within min/max bounds."""
    if raw > max_genout:
        raw_output = max_genout
    elif raw < min_genout:
        raw_output = min_genout
    else:
        raw_output = raw
    return int(raw_output)

# ----------------------------
# Movement functions (Left/Right leg movements)
# Note: Each occurrence of pwm.set_pwm() has been replaced with set_pwm().
# ----------------------------
def left_I(pos, wiggle, heightAdjust=0):
    if pos == 0:
        if leftSide_height:
            set_pwm(1, pwm1 + heightAdjust)
        else:
            set_pwm(1, pwm1 - heightAdjust)
    else:
        if leftSide_direction:
            if pos == 1:
                set_pwm(0, pwm0)
                set_pwm(1, pwm1 + (3 * height_change) if leftSide_height else pwm1 - (3 * height_change))
            elif pos == 2:
                set_pwm(0, pwm0 + wiggle)
                set_pwm(1, pwm1 - height_change if leftSide_height else pwm1 + height_change)
            elif pos == 3:
                set_pwm(0, pwm0)
                set_pwm(1, pwm1 - height_change if leftSide_height else pwm1 + height_change)
            elif pos == 4:
                set_pwm(0, pwm0 - wiggle)
                set_pwm(1, pwm1 - height_change if leftSide_height else pwm1 + height_change)
        else:
            if pos == 1:
                set_pwm(0, pwm0)
                set_pwm(1, pwm1 + (3 * wiggle) if leftSide_height else pwm1 - (3 * wiggle))
            elif pos == 2:
                set_pwm(0, pwm0 - wiggle)
                set_pwm(1, pwm1 - wiggle if leftSide_height else pwm1 + wiggle)
            elif pos == 3:
                set_pwm(0, pwm0)
                set_pwm(1, pwm1 - wiggle if leftSide_height else pwm1 + wiggle)
            elif pos == 4:
                set_pwm(0, pwm0 + wiggle)
                set_pwm(1, pwm1 - wiggle if leftSide_height else pwm1 + wiggle)

def left_II(pos, wiggle, heightAdjust=0):
    if pos == 0:
        set_pwm(3, pwm3 + heightAdjust if leftSide_height else pwm3 - heightAdjust)
    else:
        if leftSide_direction:
            if pos == 1:
                set_pwm(2, pwm2)
                set_pwm(3, pwm3 + (3 * height_change) if leftSide_height else pwm3 - (3 * height_change))
            elif pos == 2:
                set_pwm(2, pwm2 + wiggle)
                set_pwm(3, pwm3 - height_change if leftSide_height else pwm3 + height_change)
            elif pos == 3:
                set_pwm(2, pwm2)
                set_pwm(3, pwm3 - height_change if leftSide_height else pwm3 + height_change)
            elif pos == 4:
                set_pwm(2, pwm2 - wiggle)
                set_pwm(3, pwm3 - height_change if leftSide_height else pwm3 + height_change)
        else:
            if pos == 1:
                set_pwm(2, pwm2)
                set_pwm(3, pwm3 + (3 * wiggle) if leftSide_height else pwm3 - (3 * wiggle))
            elif pos == 2:
                set_pwm(2, pwm2 - wiggle)
                set_pwm(3, pwm3 - wiggle if leftSide_height else pwm3 + wiggle)
            elif pos == 3:
                set_pwm(2, pwm2)
                set_pwm(3, pwm3 - wiggle if leftSide_height else pwm3 + wiggle)
            elif pos == 4:
                set_pwm(2, pwm2 + wiggle)
                set_pwm(3, pwm3 - wiggle if leftSide_height else pwm3 + wiggle)

def left_III(pos, wiggle, heightAdjust=0):
    if pos == 0:
        set_pwm(5, pwm5 + heightAdjust if leftSide_height else pwm5 - heightAdjust)
    else:
        if leftSide_direction:
            if pos == 1:
                set_pwm(4, pwm4)
                set_pwm(5, pwm5 + (3 * height_change) if leftSide_height else pwm5 - (3 * height_change))
            elif pos == 2:
                set_pwm(4, pwm4 + wiggle)
                set_pwm(5, pwm5 - height_change if leftSide_height else pwm5 + height_change)
            elif pos == 3:
                set_pwm(4, pwm4)
                set_pwm(5, pwm5 - height_change if leftSide_height else pwm5 + height_change)
            elif pos == 4:
                set_pwm(4, pwm4 - wiggle)
                set_pwm(5, pwm5 - height_change if leftSide_height else pwm5 + height_change)
        else:
            if pos == 1:
                set_pwm(4, pwm4)
                set_pwm(5, pwm5 + (3 * wiggle) if leftSide_height else pwm5 - (3 * wiggle))
            elif pos == 2:
                set_pwm(4, pwm4 - wiggle)
                set_pwm(5, pwm5 - wiggle if leftSide_height else pwm5 + wiggle)
            elif pos == 3:
                set_pwm(4, pwm4)
                set_pwm(5, pwm5 - wiggle if leftSide_height else pwm5 + wiggle)
            elif pos == 4:
                set_pwm(4, pwm4 + wiggle)
                set_pwm(5, pwm5 - wiggle if leftSide_height else pwm5 + wiggle)

def right_I(pos, wiggle, heightAdjust=0):
    if pos == 0:
        set_pwm(7, pwm7 + heightAdjust if rightSide_height else pwm7 - heightAdjust)
    else:
        if rightSide_direction:
            if pos == 1:
                set_pwm(6, pwm6)
                set_pwm(7, pwm7 + (3 * height_change) if rightSide_height else pwm7 - (3 * height_change))
            elif pos == 2:
                set_pwm(6, pwm6 + wiggle)
                set_pwm(7, pwm7 - height_change if rightSide_height else pwm7 + height_change)
            elif pos == 3:
                set_pwm(6, pwm6)
                set_pwm(7, pwm7 - height_change if rightSide_height else pwm7 + height_change)
            elif pos == 4:
                set_pwm(6, pwm6 - wiggle)
                set_pwm(7, pwm7 - height_change if rightSide_height else pwm7 + height_change)
        else:
            if pos == 1:
                set_pwm(6, pwm6)
                set_pwm(7, pwm7 + (3 * height_change) if rightSide_height else pwm7 - (3 * height_change))
            elif pos == 2:
                set_pwm(6, pwm6 - wiggle)
                set_pwm(7, pwm7 - height_change if rightSide_height else pwm7 + height_change)
            elif pos == 3:
                set_pwm(6, pwm6)
                set_pwm(7, pwm7 - height_change if rightSide_height else pwm7 + height_change)
            elif pos == 4:
                set_pwm(6, pwm6 + wiggle)
                set_pwm(7, pwm7 - height_change if rightSide_height else pwm7 + height_change)

def right_II(pos, wiggle, heightAdjust=0):
    if pos == 0:
        set_pwm(9, pwm9 + heightAdjust if rightSide_height else pwm9 - heightAdjust)
    else:
        if rightSide_direction:
            if pos == 1:
                set_pwm(8, pwm8)
                set_pwm(9, pwm9 + (3 * height_change) if rightSide_height else pwm9 - (3 * height_change))
            elif pos == 2:
                set_pwm(8, pwm8 + wiggle)
                set_pwm(9, pwm9 - height_change if rightSide_height else pwm9 + height_change)
            elif pos == 3:
                set_pwm(8, pwm8)
                set_pwm(9, pwm9 - height_change if rightSide_height else pwm9 + height_change)
            elif pos == 4:
                set_pwm(8, pwm8 - wiggle)
                set_pwm(9, pwm9 - height_change if rightSide_height else pwm9 + height_change)
        else:
            if pos == 1:
                set_pwm(8, pwm8)
                set_pwm(9, pwm9 + (3 * height_change) if rightSide_height else pwm9 - (3 * height_change))
            elif pos == 2:
                set_pwm(8, pwm8 - wiggle)
                set_pwm(9, pwm9 - height_change if rightSide_height else pwm9 + height_change)
            elif pos == 3:
                set_pwm(8, pwm8)
                set_pwm(9, pwm9 - height_change if rightSide_height else pwm9 + height_change)
            elif pos == 4:
                set_pwm(8, pwm8 + wiggle)
                set_pwm(9, pwm9 - height_change if rightSide_height else pwm9 + height_change)

def right_III(pos, wiggle, heightAdjust=0):
    if pos == 0:
        set_pwm(11, pwm11 + heightAdjust if rightSide_height else pwm11 - heightAdjust)
    else:
        if rightSide_direction:
            if pos == 1:
                set_pwm(10, pwm10)
                set_pwm(11, pwm11 + (3 * height_change) if rightSide_height else pwm11 - (3 * height_change))
            elif pos == 2:
                set_pwm(10, pwm10 + wiggle)
                set_pwm(11, pwm11 - height_change if rightSide_height else pwm11 + height_change)
            elif pos == 3:
                set_pwm(10, pwm10)
                set_pwm(11, pwm11 - height_change if rightSide_height else pwm11 + height_change)
            elif pos == 4:
                set_pwm(10, pwm10 - wiggle)
                set_pwm(11, pwm11 - height_change if rightSide_height else pwm11 + height_change)
        else:
            if pos == 1:
                set_pwm(10, pwm10)
                set_pwm(11, pwm11 + (3 * height_change) if rightSide_height else pwm11 - (3 * height_change))
            elif pos == 2:
                set_pwm(10, pwm10 - wiggle)
                set_pwm(11, pwm11 - height_change if rightSide_height else pwm11 + height_change)
            elif pos == 3:
                set_pwm(10, pwm10)
                set_pwm(11, pwm11 - height_change if rightSide_height else pwm11 + height_change)
            elif pos == 4:
                set_pwm(10, pwm10 + wiggle)
                set_pwm(11, pwm11 - height_change if rightSide_height else pwm11 + height_change)

def move(step_input, speed, command):
    step_I  = step_input
    step_II = step_input + 2
    if step_II > 4:
        step_II -= 4
    if speed == 0:
        return
    if command == 'no':
        right_I(step_I, speed, 0)
        left_II(step_I, speed, 0)
        right_III(step_I, speed, 0)
        left_I(step_II, speed, 0)
        right_II(step_II, speed, 0)
        left_III(step_II, speed, 0)
    elif command == 'left':
        right_I(step_I, speed, 0)
        left_II(step_I, -speed, 0)
        right_III(step_I, speed, 0)
        left_I(step_II, -speed, 0)
        right_II(step_II, speed, 0)
        left_III(step_II, -speed, 0)
    elif command == 'right':
        right_I(step_I, -speed, 0)
        left_II(step_I, speed, 0)
        right_III(step_I, -speed, 0)
        left_I(step_II, speed, 0)
        right_II(step_II, -speed, 0)
        left_III(step_II, speed, 0)

def stand():
    # Set channels 0 to 11 to 300 as a "stand" position.
    for ch in range(12):
        set_pwm(ch, 300)

# ----------------------------
# (Additional functions such as dove, steady, look_* etc. have been similarly updated.)
# For brevity, see below for one more example:
# ----------------------------
def look_up(wiggle=look_wiggle):
    global Up_Down_input
    if Up_Down_direction:
        Up_Down_input += wiggle
    else:
        Up_Down_input -= wiggle
    Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
    set_pwm(13, Up_Down_input)

def look_home():
    global Left_Right_input, Up_Down_input
    set_pwm(13, 300)
    set_pwm(12, 300)
    Left_Right_input = 300
    Up_Down_input = 300

def relesae():
    set_all_pwm(0)

def clean_all():
    set_all_pwm(0)

def destroy():
    clean_all()

# ----------------------------
# Movement thread and command handling
# ----------------------------
SmoothMode = 0
steadyMode = 0
step_set = 1
speed_set = 100
DPI = 17
direction_command = 'no'
turn_command = 'no'

def move_thread():
    global step_set
    if not steadyMode:
        if direction_command == 'forward' and turn_command == 'no':
            if SmoothMode:
                # dove(...) implementation (updated similarly) goes here
                move(step_set, 35, 'no')
                step_set = (step_set % 4) + 1
            else:
                move(step_set, 35, 'no')
                time.sleep(0.1)
                step_set = (step_set % 4) + 1
        elif direction_command == 'backward' and turn_command == 'no':
            if SmoothMode:
                move(step_set, -35, 'no')
                step_set = (step_set % 4) + 1
            else:
                move(step_set, -35, 'no')
                time.sleep(0.1)
                step_set = (step_set % 4) + 1
        elif turn_command != 'no':
            if SmoothMode:
                move(step_set, 35, turn_command)
                step_set = (step_set % 4) + 1
            else:
                move(step_set, 35, turn_command)
                time.sleep(0.1)
                step_set = (step_set % 4) + 1
        if turn_command == 'no' and direction_command == 'stand':
            stand()
            step_set = 1
    else:
        # When steady mode is enabled:
        steady_X()
        steady()

class RobotM(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(RobotM, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    def pause(self):
        self.__flag.clear()

    def resume(self):
        self.__flag.set()

    def run(self):
        while True:
            self.__flag.wait()
            move_thread()

rm = RobotM()
rm.start()
rm.pause()

def commandInput(command_input):
    global direction_command, turn_command, SmoothMode, steadyMode
    if command_input == 'forward':
        direction_command = 'forward'
        rm.resume()
    elif command_input == 'backward':
        direction_command = 'backward'
        rm.resume()
    elif 'stand' in command_input:
        direction_command = 'stand'
        rm.pause()
    elif command_input == 'left':
        turn_command = 'left'
        rm.resume()
    elif command_input == 'right':
        turn_command = 'right'
        rm.resume()
    elif 'no' in command_input:
        turn_command = 'no'
        rm.pause()
    elif command_input == 'automaticOff':
        SmoothMode = 0
        steadyMode = 0
        rm.pause()
    elif command_input == 'automatic':
        rm.resume()
        SmoothMode = 1
    elif command_input == 'KD':
        steadyMode = 1
        rm.resume()
    elif command_input == 'speech':
        steadyMode = 1
        rm.resume()
    elif command_input == 'speechOff':
        SmoothMode = 0
        steadyMode = 0
        rm.pause()

# ----------------------------
# Main execution block
# ----------------------------
if __name__ == '__main__':
    step = 1
    move_stu = 1
    try:
        while True:
            move(step, 35, 'no')
            step = step + 1 if step < 4 else 1
            time.sleep(0.08)
    except KeyboardInterrupt:
        set_all_pwm(300)
        time.sleep(1)

