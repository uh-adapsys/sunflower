#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
from collections import namedtuple

import logging
from threading import Thread
import math
import rospy
import time

from controller import Robot
try:
    import roslib
    import os
    path = os.path.dirname(os.path.realpath(__file__))
    roslib.load_manifest('rosController')
except:
    logger = logging.getLogger()
    if logger.handlers:
        logging.getLogger().error(
            "Unable to load roslib, fatal error", exc_info=True)
    else:
        import sys
        import traceback
        print >> sys.stderr, "Unable to load roslib, fatal error"
        print >> sys.stderr, traceback.format_exc()
    exit(1)
else:
    import actionlib
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from nav_msgs.msg import Odometry
    # from p2os_msgs.msg import SonarArray
    from rosgraph_msgs.msg import Clock
    from tf import TransformBroadcaster
    from tf.transformations import quaternion_from_euler

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

class ClockSync(Thread):

    def __init__(self, robot):
        super(ClockSync, self).__init__()
        self.getTime = robot.getTime
        self._stop = False

    def run(self):
        if "/clock" in rospy.get_published_topics():
            # Another module is publishing clock
            return
        
        try:
            # ROS Version >= Hydro
            clockPublisher = rospy.Publisher("/clock", Clock, queue_size=2)
        except:
            clockPublisher = rospy.Publisher("/clock", Clock)

        self._stop = False
        starttime = time.time()
        while not rospy.is_shutdown() and not self._stop:
            rosTime = rospy.Time(self.getTime() + starttime)
            clock = Clock(clock=rosTime)
            clockPublisher.publish(clock)
            time.sleep(0.001)

    def stop(self):
        self._stop = True

class Person(Robot):

    _actionHandles = {}
    Location = namedtuple('Location', ['x', 'y', 'theta', 'timestamp'])

    # TODO: These should be in a config file
    # Speed limits from navigation files
    _translationSpeed = [-0.2, 0.4]
    _rotationSpeed = [-0.8, 0.8]
    
    LINEAR_RATE = math.pi / 2  # [rad/s]
    WHEEL_DIAMETER = 0.1
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    AXEL_LENGTH = 0.36
    WHEEL_ROTATION = AXEL_LENGTH / WHEEL_DIAMETER
    MAX_WHEEL_SPEED = 5.24
    
    def __init__(self, name):
        super(Person, self).__init__()
        self._time_step = int(self.getBasicTimeStep())
        self._logger = logging.getLogger(self.__class__.__name__)
        self._location = None
        self._lastLocation = None
        self._leds = {}
        self._sensors = {}
        self.initialise()

    def _updateLocation(self):
        if self._sensors.get('gps', None) and self._sensors.get('compass', None):
            lX, lY, lZ = self._sensors['gps'].getValues()
            wX, _, wZ = self._sensors['compass'].getValues()

            # http://www.cyberbotics.com/reference/section3.13.php
            bearing = math.atan2(wX, wZ) - (math.pi / 2)
            x, y, _, rotation = self.webotsToRos(lX, lY, lZ, bearing)

            self._lastLocation = self._location
            self._location = Person.Location(x, y, rotation, self.getTime())  

    def webotsToRos(self, x, y, z, theta):
        rX = -z
        rY = -x
        rZ = y
        theta = -1 * theta
        return (rX, rY, rZ, theta)
    
    def run(self):
        while self.step(self._time_step) != -1:
            self._updateLocation()
            self.setlight([0,1,0])
            rate = 3
            [w.setVelocity(rate) for w in self._rightWheels]
            [w.setVelocity(rate) for w in self._leftWheels]
            time.sleep(0.0001)

    def initialise(self):
        self._leftWheels = [self.getMotor("fl_wheel"), self.getMotor("rl_wheel"), ]
        self._rightWheels = [self.getMotor("fr_wheel"), self.getMotor("rr_wheel"), ]
        wheels = []
        wheels.extend(self._leftWheels)
        wheels.extend(self._rightWheels)
        for wheel in wheels:
            wheel.setPosition(float('+inf'))
            wheel.setVelocity(0)

        self._sensors['gps'] = self.getGPS("gps")
        self._sensors['gps'].enable(self._time_step)

        self._sensors['compass'] = self.getCompass("compass")
        self._sensors['compass'].enable(self._time_step)

        self._leds['body'] = self.getLED("light")

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
            if self._leds['body']:
                self._leds['body'].set(colorIndex)
                return True
            else:
                self._logger.error("Unable to set color.  Body LED not found.")
                return False
        except Exception:
            self._logger.error("Error setting color to: %s" % (color), exc_info=True)
            return False

    def moveBase(self, positions):
        maxTransNeg, maxTransPos = self._translationSpeed
        maxRotNeg, maxRotPos = self._rotationSpeed

        rotation = round(positions[0], 4)
        linear = round(positions[1], 4)

        if not isinstance(rotation, (int, float)):
            self._logger.error("Non-numeric rotation in list, aborting moveBase")
            return _states['ABORTED']
        elif not isinstance(linear, (int, float)):
            self._logger.error("Non-numeric translation in list, aborting moveBase")
            return _states['ABORTED']
#         if linear > maxTransPos:
#             self._logger.error(
#                 "Maximal relative translation step exceeded(max: %sm, requested: %sm), "
#                 "aborting moveBase" % (maxTransPos, linear))
#             return _states['ABORTED']
#         if linear < maxTransNeg:
#             self._logger.error(
#                 "Minimal relative translation step exceeded(min: %sm, requested: %sm), "
#                 "aborting moveBase" % (maxTransNeg, linear))
#             return _states['ABORTED']
#         if rotation > maxRotPos:
#             self._logger.error(
#                 "Maximal relative rotation step exceeded(max: %srad, requested: %sm), "
#                 "aborting moveBase" % (maxRotPos, rotation))
#             return _states['ABORTED']
#         if rotation < maxRotNeg:
#             self._logger.error(
#                 "Maximal relative rotation step exceeded(max: %srad, requested: %sm), "
#                 "aborting moveBase" % (maxRotNeg, rotation))
#             return _states['ABORTED']

        rotRads = rotation * Person.WHEEL_ROTATION
        linearRads = linear / Person.WHEEL_RADIUS

        leftDuration = (rotRads + linearRads) / Person.LINEAR_RATE
        rightDuration = ((-1 * rotRads) + linearRads) / Person.LINEAR_RATE

        leftRate = Person.LINEAR_RATE
        rightRate = Person.LINEAR_RATE
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

        self._logger.info("Setting rates: L=%s, R=%s" % (leftRate, rightRate))
        duration = max(leftDuration, rightDuration)
        [w.setVelocity(rightRate) for w in self._rightWheels]
        [w.setVelocity(leftRate) for w in self._leftWheels]
        time.sleep(duration)
        [w.setVelocity(0) for w in self._rightWheels]
        [w.setVelocity(0) for w in self._leftWheels]

        return _states['SUCCEEDED']

    def cmdvel_cb(self, msg):        
        # Get in-range linear and angular values
        linear = min(max(msg.linear.x, self._translationSpeed[0]), self._translationSpeed[1])
        angular = min(max(msg.angular.z, self._rotationSpeed[0]), self._rotationSpeed[1])
        
        linearRads = linear / Person.WHEEL_RADIUS
        rotRads = angular * Person.WHEEL_ROTATION
        
        # Get in-range hard limits
        right = max(min(linearRads + rotRads, Person.MAX_WHEEL_SPEED), -Person.MAX_WHEEL_SPEED)
        left = max(min(linearRads - rotRads, Person.MAX_WHEEL_SPEED), -Person.MAX_WHEEL_SPEED)
        
        [w.setVelocity(right) for w in self._rightWheels]
        [w.setVelocity(left) for w in self._leftWheels]

    def navigate(self, goal, positions):
        pose = PoseStamped()
        pose.header.stamp = self._rosTime
        pose.header.frame_id = "/map"
        pose.pose.position.x = positions[0]
        pose.pose.position.y = positions[1]
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, positions[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        client_goal = MoveBaseGoal()
        client_goal.target_pose = pose

        client.wait_for_server()
        rospy.loginfo("%s: Navigating to (%s, %s, %s)",
                      self._action_name,
                      positions[0],
                      positions[1],
                      positions[2])

        handle = _ActionHandle(client)
        client.send_goal(client_goal)
        handle.wait()
        return handle.result


class _ActionHandle(object):
    # ------------------- action_handle section ------------------- #
    # Action handle class.
    #
    # The action handle is used to implement asynchronous behaviour within the
    # script.
    def __init__(self, simpleActionClient):
        # Initialises the action handle.
        self._client = simpleActionClient
        self._waiting = False
        self._result = None

    @property
    def result(self):
        if self._waiting:
            return None

        return self._result

    def wait(self, duration=None):
        t = self.waitAsync(duration)
        t.join()

    def waitAsync(self, duration=None):
        thread = Thread(target=self._wait_for_finished, args=(duration,))
        thread.setDaemon(True)
        thread.start()
        return thread

    def _wait_for_finished(self, duration):
        self._waiting = True
        if duration is None:
            self._client.wait_for_result()
        else:
            self._client.wait_for_result(rospy.Duration(duration))

        self._result = self._client.get_state()
        self._waiting = False

    def cancel(self):
        self._client.cancel_all_goals()


if __name__ == '__main__':
    logging.basicConfig()
    rospy.init_node('people_controller')
    sf = Person(rospy.get_name())
    sf.run()
