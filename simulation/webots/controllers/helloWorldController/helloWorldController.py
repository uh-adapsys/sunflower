#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
from collections import namedtuple
import logging
import math
import time

from controller import Robot


_states = {
    0: 'PENDING',
    'PENDING': 0,
    1: 'ACTIVE',
    'ACTIVE': 1,
    2: 'PREEMPTED',
    'PREEMPTED': 2,
    3: 'SUCCEEDED',
    'SUCCEEDED': 3,
    4: 'ABORTED',
    'ABORTED': 4,
    5: 'REJECTED',
    'REJECTED': 5,
    6: 'PREEMPTING',
    'PREEMPTING': 6,
    7: 'RECALLING',
    'RECALLING': 7,
    8: 'RECALLED',
    'RECALLED': 8,
    9: 'LOST',
    'LOST': 9,
}


class Sunflower(Robot):

    _actionHandles = {}
    Location = namedtuple('Location', ['x', 'y', 'theta', 'timestamp'])
    DistanceScan = namedtuple('DistanceScan', [
                              'min_angle',
                              'max_angle',
                              'min_range',
                              'max_range',
                              'scan_time',
                              'ranges'])

    def __init__(self, name):
        super(Sunflower, self).__init__()
        self.initialise()
        self._location = None
        self._lastLocation = None
        self._sensorValues = None
        self._rosTime = None

    def _updateLocation(self):
        rawLocation = self._gps.getValues()
        rawHeading = self._compass.getValues()

        # http://www.cyberbotics.com/reference/section3.13.php
        rotation = math.atan2(rawHeading[0], rawHeading[2])
        # WeBots and ROS have
        x = round(rawLocation[0], 3)
        y = round(rawLocation[2], 3)
        # Adjust for differences between WeBots and ROS world models
        rotation = -1 * rotation
        x = -1 * x
        self._lastLocation = self._location
        self._location = Sunflower.Location(x, y, rotation, self.getTime())

    def _updateSonar(self):
        self._sensorValues = map(lambda x: x.getValue(), self._sensors)

    def _updateLaser(self):
        fov = self._frontLaser.getFov()
        ranges = self._frontLaser.getRangeImage()
        ranges.reverse()
        maxRange = self._frontLaser.getMaxRange()
        sampleRate = self._frontLaser.getSamplingPeriod() / 1000

        self._laserValues = Sunflower.DistanceScan(
            fov / -2,
            fov / 2,
            0,
            maxRange,
            sampleRate,
            ranges
        )

    def run(self):
        while True:
            if self.step(self._time_step) == -1:
                break

            self._updateLocation()
            self._updateSonar()
            self._updateLaser()

    def initialise(self):
        numBaseLeds = 3
        numSonarSensors = 16

        self._time_step = int(self.getBasicTimeStep())

        self._leftWheel = self.getMotor("left_wheel")
        self._leftWheel.setPosition(float('+inf'))
        self._leftWheel.setVelocity(0.0)

        self._rightWheel = self.getMotor("right_wheel")
        self._rightWheel.setPosition(float('+inf'))
        self._rightWheel.setVelocity(0.0)

        self._servos = {
                ("tray", self.getMotor("tray")),
                ("neck_lower", self.getMotor("neck_lower")),
                ("neck_upper", self.getMotor("neck_upper")),
                ("head_tilt", self.getMotor("head_tilt")),
                ("head_pan", self.getMotor("head_pan")),
        }

        self._frontLaser = self.getCamera("front_laser")
        self._frontLaser.enable(self._time_step)

        self._gps = self.getGPS("gps")
        self._gps.enable(self._time_step)

        self._compass = self.getCompass("compass")
        self._compass.enable(self._time_step)

        self._camera = self.getCamera("head_camera")
        if self._camera:
            self._camera.enable(self._time_step)

        self._bodyLED = self.getLED("light")

        self._baseLEDs = []
        for i in range(0, numBaseLeds):
            led = self.getLED("red_led%s" % (i + 1))
            led.set(0)
            self._baseLEDs.append(led)

        self._sensors = []
        self._sensorValues = []
        for i in range(0, numSonarSensors):
            sensor = self.getDistanceSensor("so%s" % i)
            sensor.enable(self._time_step)
            self._sensors.append(sensor)

    def park(self):
        pass

    def init(self, name):
        if name == 'base' or name == 'base_direct':
            pass

        return True

    def setlight(self, color):
        # Sunflower hardware only supports on/off states for RGB array
        # Webots selects the color as an array index of available colors
        # 3-bit color array is arranged in ascending binary order
        try:
            r = 0x4 if color[0] else 0
            g = 0x2 if color[1] else 0
            b = 0x1 if color[2] else 0
            # Webots color array is 1-indexed
            colorIndex = r + g + b + 1
            self._bodyLED.set(colorIndex)
            return True
        except Exception:
            logging.getLogger().error("Error setting color to: %s" % (color), exc_info=True)
            return False

    def moveBase(self, rotation, linear):
        # TODO: These should be in a config file
        maxTrans = 1.5
        maxRot = 2 * math.pi

        LINEAR_RATE = math.pi / 2  # [rad/s]
        # WHEEL_SIZE = 0.195  # [m] From the manual
        WHEEL_SIZE = 0.1918986  # [m] From Webots Definition
        # BASE_SIZE = 0.3810  # [m] From the manual
        BASE_SIZE = 0.33  # [m] From WeBots Definition
        WHEEL_ROTATION = BASE_SIZE / WHEEL_SIZE

        if not isinstance(rotation, (int, float)):
            logging.getLogger().error("Non-numeric rotation in list, aborting moveBase")
            return _states['ABORTED']
        elif not isinstance(linear, (int, float)):
            logging.getLogger().error("Non-numeric translation in list, aborting moveBase")
            return _states['ABORTED']
        if abs(linear) > maxTrans:
            logging.getLogger().error(
                "Maximal relative translation step exceeded(max: %sm), "
                "aborting moveBase" % maxTrans)
            return _states['ABORTED']
        if abs(rotation) > maxRot:
            logging.getLogger().error(
                "Maximal relative rotation step exceeded(max: %srad), "
                "aborting moveBase" % maxRot)
            return _states['ABORTED']

        rotRads = rotation * WHEEL_ROTATION
        linearRads = (2 * linear) / (WHEEL_SIZE)

        leftDuration = (rotRads + linearRads) / LINEAR_RATE
        rightDuration = ((-1 * rotRads) + linearRads) / LINEAR_RATE

        leftRate = LINEAR_RATE
        rightRate = LINEAR_RATE
        if leftDuration < 0:
            leftRate = -1 * leftRate
            leftDuration = abs(leftDuration)
        if rightDuration < 0:
            rightRate = -1 * rightRate
            rightDuration = abs(rightDuration)
        if leftDuration != rightDuration:
            if leftDuration < rightDuration:
                leftRate = leftRate * (leftDuration / rightDuration)
            else:
                rightRate = rightRate * (rightDuration / leftDuration)

        duration = max(leftDuration, rightDuration)
        start_time = self.getTime()
        end_time = start_time + duration
        while True:
            if self._as.is_preempt_requested():
                logging.getLogger().error('%s: Preempted' % self._action_name)
                self._rightWheel.setVelocity(0)
                self._leftWheel.setVelocity(0)
                # self._as.set_preempted()
                return _states['PREEMPTED']

            self._rightWheel.setVelocity(rightRate)
            self._leftWheel.setVelocity(leftRate)

            if self.getTime() >= end_time:
                break

        self._rightWheel.setVelocity(0)
        self._leftWheel.setVelocity(0)

        return _states['SUCCEEDED']

    def moveJoints(self, jointName, position):
        if jointName not in self._servos:
            logging.getLogger().error('Undefined joint %s', jointName)
            return _states['ABORTED']
        self._servos[jointName].set(position)

        return _states['SUCCEEDED']

if __name__ == '__main__':
    sf = Sunflower("Sunflower")
    sf.run()
