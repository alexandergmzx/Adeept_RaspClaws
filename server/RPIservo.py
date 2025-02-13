#!/usr/bin/env python3
# File name   : servo.py
# Description : Control Servos with the updated Adafruit PCA9685 library
# Author      : William (modified 2025)
from __future__ import division
import time
import RPi.GPIO as GPIO
import sys
# New library imports
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import threading
import random

# Initialize I2C and create PCA9685 object
i2c = busio.I2C(SCL, SDA)
pwm = PCA9685(i2c)
pwm.frequency = 50

def convert_to_duty_cycle(pulse):
    """
    Convert a pulse value (expected in the 0–4096 range)
    to a 16-bit duty cycle value (0–65535).
    Adjust this mapping as needed for your servo pulse calibration.
    """
    # Ensure pulse is within a valid range:
    if pulse < 0:
        pulse = 0
    elif pulse > 4096:
        pulse = 4096
    return int(pulse * 65535 / 4096)

# Initial pulse values for each channel
init_pwm0 = 300
init_pwm1 = 300
init_pwm2 = 300
init_pwm3 = 300

init_pwm4 = 300
init_pwm5 = 300
init_pwm6 = 300
init_pwm7 = 300

init_pwm8 = 300
init_pwm9 = 300
init_pwm10 = 300
init_pwm11 = 300

init_pwm12 = 300
init_pwm13 = 300
init_pwm14 = 300
init_pwm15 = 300

class ServoCtrl(threading.Thread):

    def __init__(self, *args, **kwargs):
        # Servo directions (1 or -1)
        self.sc_direction = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.initPos = [init_pwm0, init_pwm1, init_pwm2, init_pwm3,
                        init_pwm4, init_pwm5, init_pwm6, init_pwm7,
                        init_pwm8, init_pwm9, init_pwm10, init_pwm11,
                        init_pwm12, init_pwm13, init_pwm14, init_pwm15]
        self.goalPos = [300] * 16
        self.nowPos  = [300] * 16
        self.bufferPos  = [300.0] * 16
        self.lastPos = [300] * 16
        self.ingGoal = [300] * 16
        self.maxPos  = [560] * 16
        self.minPos  = [100] * 16
        self.scSpeed = [0] * 16

        self.ctrlRangeMax = 560
        self.ctrlRangeMin = 100
        self.angleRange = 180

        # Modes: 'init', 'auto', 'certain', 'quick', 'wiggle'
        self.scMode = 'auto'
        self.scTime = 2.0
        self.scSteps = 30

        self.scDelay = 0.037
        self.scMoveTime = 0.037

        self.goalUpdate = 0
        self.wiggleID = 0
        self.wiggleDirection = 1

        super(ServoCtrl, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    def pause(self):
        print('......................pause..........................')
        self.__flag.clear()

    def resume(self):
        print('resume')
        self.__flag.set()

    def moveInit(self):
        self.scMode = 'init'
        for i in range(16):
            pwm.channels[i].duty_cycle = convert_to_duty_cycle(self.initPos[i])
            self.lastPos[i] = self.initPos[i]
            self.nowPos[i] = self.initPos[i]
            self.bufferPos[i] = float(self.initPos[i])
            self.goalPos[i] = self.initPos[i]
        self.pause()

    def initConfig(self, ID, initInput, moveTo):
        if self.minPos[ID] < initInput < self.maxPos[ID]:
            self.initPos[ID] = initInput
            if moveTo:
                pwm.channels[ID].duty_cycle = convert_to_duty_cycle(self.initPos[ID])
        else:
            print('initPos Value Error.')

    def moveServoInit(self, ID):
        self.scMode = 'init'
        for i in range(len(ID)):
            channel = ID[i]
            pwm.channels[channel].duty_cycle = convert_to_duty_cycle(self.initPos[channel])
            self.lastPos[channel] = self.initPos[channel]
            self.nowPos[channel] = self.initPos[channel]
            self.bufferPos[channel] = float(self.initPos[channel])
            self.goalPos[channel] = self.initPos[channel]
        self.pause()

    def posUpdate(self):
        self.goalUpdate = 1
        for i in range(16):
            self.lastPos[i] = self.nowPos[i]
        self.goalUpdate = 0

    def speedUpdate(self, IDinput, speedInput):
        for i in range(len(IDinput)):
            self.scSpeed[IDinput[i]] = speedInput[i]

    def moveAuto(self):
        for i in range(16):
            self.ingGoal[i] = self.goalPos[i]

        for i in range(self.scSteps):
            for dc in range(16):
                if not self.goalUpdate:
                    self.nowPos[dc] = int(round(self.lastPos[dc] + ((self.goalPos[dc] - self.lastPos[dc]) / self.scSteps * (i + 1)), 0))
                    pwm.channels[dc].duty_cycle = convert_to_duty_cycle(self.nowPos[dc])
                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    time.sleep(self.scTime / self.scSteps)
                    return 1
            time.sleep((self.scTime / self.scSteps) - self.scMoveTime)

        self.posUpdate()
        self.pause()
        return 0

    def moveCert(self):
        for i in range(16):
            self.ingGoal[i] = self.goalPos[i]
            self.bufferPos[i] = self.lastPos[i]

        while self.nowPos != self.goalPos:
            for i in range(16):
                if self.lastPos[i] < self.goalPos[i]:
                    self.bufferPos[i] += self.pwmGenOut(self.scSpeed[i]) / (1 / self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow > self.goalPos[i]:
                        newNow = self.goalPos[i]
                    self.nowPos[i] = newNow
                elif self.lastPos[i] > self.goalPos[i]:
                    self.bufferPos[i] -= self.pwmGenOut(self.scSpeed[i]) / (1 / self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow < self.goalPos[i]:
                        newNow = self.goalPos[i]
                    self.nowPos[i] = newNow

                if not self.goalUpdate:
                    pwm.channels[i].duty_cycle = convert_to_duty_cycle(self.nowPos[i])

                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    return 1
            self.posUpdate()
            time.sleep(self.scDelay - self.scMoveTime)
        else:
            self.pause()
            return 0

    def pwmGenOut(self, angleInput):
        return int(round(((self.ctrlRangeMax - self.ctrlRangeMin) / self.angleRange * angleInput), 0))

    def setAutoTime(self, autoSpeedSet):
        self.scTime = autoSpeedSet

    def setDelay(self, delaySet):
        self.scDelay = delaySet

    def autoSpeed(self, ID, angleInput):
        self.scMode = 'auto'
        self.goalUpdate = 1
        for i in range(len(ID)):
            channel = ID[i]
            newGoal = self.initPos[channel] + self.pwmGenOut(angleInput[i]) * self.sc_direction[channel]
            if newGoal > self.maxPos[channel]:
                newGoal = self.maxPos[channel]
            elif newGoal < self.minPos[channel]:
                newGoal = self.minPos[channel]
            self.goalPos[channel] = newGoal
        self.goalUpdate = 0
        self.resume()

    def certSpeed(self, ID, angleInput, speedSet):
        self.scMode = 'certain'
        self.goalUpdate = 1
        for i in range(len(ID)):
            channel = ID[i]
            newGoal = self.initPos[channel] + self.pwmGenOut(angleInput[i]) * self.sc_direction[channel]
            if newGoal > self.maxPos[channel]:
                newGoal = self.maxPos[channel]
            elif newGoal < self.minPos[channel]:
                newGoal = self.minPos[channel]
            self.goalPos[channel] = newGoal
        self.speedUpdate(ID, speedSet)
        self.goalUpdate = 0
        self.resume()

    def moveWiggle(self):
        self.bufferPos[self.wiggleID] += (
            self.wiggleDirection *
            self.sc_direction[self.wiggleID] *
            self.pwmGenOut(self.scSpeed[self.wiggleID])
        ) / (1 / self.scDelay)
        newNow = int(round(self.bufferPos[self.wiggleID], 0))
        if self.bufferPos[self.wiggleID] > self.maxPos[self.wiggleID]:
            self.bufferPos[self.wiggleID] = self.maxPos[self.wiggleID]
        elif self.bufferPos[self.wiggleID] < self.minPos[self.wiggleID]:
            self.bufferPos[self.wiggleID] = self.minPos[self.wiggleID]
        self.nowPos[self.wiggleID] = newNow
        self.lastPos[self.wiggleID] = newNow
        if self.minPos[self.wiggleID] < self.bufferPos[self.wiggleID] < self.maxPos[self.wiggleID]:
            pwm.channels[self.wiggleID].duty_cycle = convert_to_duty_cycle(self.nowPos[self.wiggleID])
        else:
            self.stopWiggle()
        time.sleep(self.scDelay - self.scMoveTime)

    def stopWiggle(self):
        self.pause()
        self.posUpdate()

    def singleServo(self, ID, direcInput, speedSet):
        self.wiggleID = ID
        self.wiggleDirection = direcInput
        self.scSpeed[ID] = speedSet
        self.scMode = 'wiggle'
        self.posUpdate()
        self.resume()

    def moveAngle(self, ID, angleInput):
        self.nowPos[ID] = int(self.initPos[ID] + self.sc_direction[ID] * self.pwmGenOut(angleInput))
        if self.nowPos[ID] > self.maxPos[ID]:
            self.nowPos[ID] = self.maxPos[ID]
        elif self.nowPos[ID] < self.minPos[ID]:
            self.nowPos[ID] = self.minPos[ID]
        self.lastPos[ID] = self.nowPos[ID]
        pwm.channels[ID].duty_cycle = convert_to_duty_cycle(self.nowPos[ID])

    def scMove(self):
        if self.scMode == 'init':
            self.moveInit()
        elif self.scMode == 'auto':
            self.moveAuto()
        elif self.scMode == 'certain':
            self.moveCert()
        elif self.scMode == 'wiggle':
            self.moveWiggle()

    def setPWM(self, ID, PWM_input):
        self.lastPos[ID] = PWM_input
        self.nowPos[ID] = PWM_input
        self.bufferPos[ID] = float(PWM_input)
        self.goalPos[ID] = PWM_input
        pwm.channels[ID].duty_cycle = convert_to_duty_cycle(PWM_input)
        self.pause()

    def run(self):
        while True:
            self.__flag.wait()
            self.scMove()

if __name__ == '__main__':
    sc = ServoCtrl()
    sc.start()
    while True:
        sc.moveAngle(0, (random.random() * 100 - 50))
        time.sleep(1)
        sc.moveAngle(1, (random.random() * 100 - 50))
        time.sleep(1)
        # Uncomment the following for additional testing:
        # sc.singleServo(0, 1, 5)
        # time.sleep(6)
        # sc.singleServo(0, -1, 30)
        # time.sleep(1)

