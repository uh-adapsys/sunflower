#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
try:
    import roslib
    roslib.load_manifest('rosController')
except:
    import logging
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

from collections import namedtuple
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from p2os_msgs.msg import SonarArray
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from threading import Thread
import actionlib
import math
import rospy
import time

from controller import Robot
import sf_controller_msgs.msg


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
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            sf_controller_msgs.msg.SunflowerAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self._as.start()

        rospy.loginfo(
            "Started Sunflower Controller ActionServer on topic %s",
            self._action_name)
        self._feedback = sf_controller_msgs.msg.SunflowerFeedback()
        self._result = sf_controller_msgs.msg.SunflowerResult()
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

    def _publishOdomTransform(self, odomPublisher):
        if self._location:
            odomPublisher.sendTransform(
                (self._location.x, self._location.y, 0),
                quaternion_from_euler(0, 0, self._location.theta),
                self._rosTime,
                'base_link',
                'odom')

    def _publishLocationTransform(self, locationPublisher):
        if self._location:
            locationPublisher.sendTransform(
                (self._location.x, self._location.y, 0),
                quaternion_from_euler(0, 0, self._location.theta),
                self._rosTime,
                'odom',
                'map',)

    def _publishLaserTransform(self, laserPublisher):
        if self._location:
            #             laserPublisher.sendTransform(
            #                 (0.15, 0.0, 0.245),
            #                 quaternion_from_euler(0, 0, 0),
            #                 self._rosTime,
            #                 'base_laser_front_link',
            #                 'base_link')
            laserPublisher.sendTransform(
                (0.0, 0.0, 0.0),
                quaternion_from_euler(0, 0, 0),
                self._rosTime,
                'scan_front',
                'base_laser_front_link')

    def _publishOdom(self, odomPublisher):
        if self._location and self._lastLocation:
            dt = (
                self._location.timestamp - self._lastLocation.timestamp) / 1000

            msg = Odometry()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = 'odom'

            msg.pose.pose.position.x = self._location.x
            msg.pose.pose.position.y = self._location.y
            msg.pose.pose.position.z = 0

            orientation = quaternion_from_euler(
                0, 0, self._location.theta)
            msg.pose.pose.orientation.x = orientation[0]
            msg.pose.pose.orientation.y = orientation[1]
            msg.pose.pose.orientation.z = orientation[2]
            msg.pose.pose.orientation.w = orientation[3]

            msg.child_frame_id = "base_link"
            msg.twist.twist.linear.x = (
                self._location.x - self._lastLocation.x) / dt
            msg.twist.twist.linear.y = (
                self._location.y - self._lastLocation.y) / dt
            msg.twist.twist.angular.x = (
                self._location.theta - self._lastLocation.theta) / dt

            odomPublisher.publish(msg)

    def _publishPose(self, posePublisher):
        if self._location:
            msg = PoseStamped()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = 'map'

            msg.pose.position.x = self._location.x
            msg.pose.position.y = self._location.y
            msg.pose.position.z = 0

            orientation = quaternion_from_euler(0, 0, self._location.theta)
            msg.pose.orientation.x = orientation[0]
            msg.pose.orientation.y = orientation[1]
            msg.pose.orientation.z = orientation[2]
            msg.pose.orientation.w = orientation[3]

            posePublisher.publish(msg)

    def _publishSonar(self, sonarPublisher):
        if self._sensorValues:
            msg = SonarArray()
            msg.header.stamp = self._rosTime

            msg.ranges = self._sensorValues
            msg.ranges_count = len(self._sensorValues)
            sonarPublisher.publish(msg)

    def _publishClock(self, clockPublisher):
        if clockPublisher:
            clock = Clock(clock=self._rosTime)
            clockPublisher.publish(clock)

    def _publishLaser(self, laserPublisher):
        if self._laserValues:
            laser_frequency = 40
            msg = LaserScan()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = 'scan_front'

            msg.ranges = self._laserValues.ranges
            msg.angle_min = self._laserValues.min_angle
            msg.angle_max = self._laserValues.max_angle
            msg.angle_increment = abs(
                msg.angle_max - msg.angle_min) / len(msg.ranges)
            msg.range_min = self._laserValues.min_range
            msg.range_max = self._laserValues.max_range
            msg.scan_time = self._time_step
            msg.time_increment = (msg.scan_time / laser_frequency /
                                  len(self._laserValues.ranges))
            laserPublisher.publish(msg)

    def run(self):
        try:
            # ROS Version >= Hydro
            sonarPublisher = rospy.Publisher(
                "/sonar", SonarArray, queue_size=2)
            odomPublisher = rospy.Publisher("/odom", Odometry, queue_size=2)
            posePublisher = rospy.Publisher("/pose", PoseStamped, queue_size=2)
            laserPublisher = rospy.Publisher(
                "/scan_front", LaserScan, queue_size=2)
            clockPublisher = rospy.Publisher("/clock", Clock, queue_size=2)
        except:
            sonarPublisher = rospy.Publisher("/sonar", SonarArray)
            odomPublisher = rospy.Publisher("/odom", Odometry)
            posePublisher = rospy.Publisher("/pose", PoseStamped)
            laserPublisher = rospy.Publisher("/scan_front", LaserScan)
            clockPublisher = rospy.Publisher("/clock", Clock)
        odomTransform = TransformBroadcaster()
        locationTransform = TransformBroadcaster()
        laserTransform = TransformBroadcaster()

        # Probably something wrong elsewhere, but we seem to need to publish
        # map->odom transform once to get sf_navigation to load
        self._publishLocationTransform(locationTransform)

        while not rospy.is_shutdown():
            if self.step(self._time_step) == -1:
                break

            self._rosTime = rospy.Time(self.getTime())

            self._updateLocation()
            self._updateSonar()
            self._updateLaser()
            self._publishClock(clockPublisher)
            self._publishPose(posePublisher)
            self._publishOdom(odomPublisher)
            self._publishOdomTransform(odomTransform)
            # published by robot_joint_publisher
            self._publishLaserTransform(laserTransform)
            # Published by sf_navigation
            # self._publishLocationTransform(locationTransform)
            self._publishSonar(sonarPublisher)
            self._publishLaser(laserPublisher)

            # It appears that we have to call sleep for ROS to process messages
            time.sleep(0)

    def initialise(self):
        numLeds = 3
        numSonarSensors = 16

        self._time_step = int(self.getBasicTimeStep())

        self._leftWheel = self.getMotor("left_wheel")
        self._leftWheel.setPosition(float('+inf'))
        self._leftWheel.setVelocity(0.0)

        self._rightWheel = self.getMotor("right_wheel")
        self._rightWheel.setPosition(float('+inf'))
        self._rightWheel.setVelocity(0.0)

        self._tray = self.getMotor("tray")

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
        for i in range(0, numLeds):
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

    def execute_cb(self, goal):
        rospy.loginfo("Got goal: %s" % goal)
        if goal.component == 'light':
            result = self.setlight(goal.jointPositions)
        elif goal.action == 'move':
            result = self.move(goal)
        elif goal.action == 'init':
            result = self.init(goal.component)
        elif goal.action == 'stop':
            self.stop(goal.component)
            result = True
        elif goal.action == 'park':
            self.park()
            result = True
        else:
            rospy.logwarn("Unknown action %s", goal.action)
            self._result.result = -1
            self._as.set_aborted(self._result)

        if result:
            self._result.result = 0
            self._as.set_succeeded(self._result)
        else:
            self._result.result = -1
            self._as.set_aborted(self._result)

    def init(self, name):
        if name == 'base' or name == 'base_direct':
            pass

        return True

    def stop(self, name):
        rospy.loginfo("%s: Stopping %s", self._action_name, name)
        if name == 'base':
            client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            client.wait_for_server()
            client.cancel_all_goals()
        else:
            pass

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
            rospy.logerr("Error setting color to: %s" % (color), exc_info=True)
            return False

    def move(self, goal):
        joints = goal.jointPositions

        if(goal.namedPosition != '' and goal.namedPosition is not None):
            param = '/sf_controller/' + \
                goal.component + '/' + goal.namedPosition
            if(rospy.has_param(param)):
                joints = rospy.get_param(param)[0]

        rospy.loginfo("%s: Setting %s to %s",
                      self._action_name,
                      goal.component,
                      goal.namedPosition or joints)

        try:
            if goal.component == 'base':
                result = self.navigate(goal, joints)
            elif goal.component == 'base_direct':
                result = self.moveBase(goal, joints)
            else:
                result = self.moveJoints(goal, joints)

            rospy.logdebug("%s: '%s to %s' Result:%s",
                           self._action_name,
                           goal.component,
                           goal.namedPosition or joints,
                           result)
        except Exception as e:
            rospy.logerr("Error occurred: %s" % e)
            return False

        return result == _states['SUCCEEDED']

    def moveBase(self, goal, positions):
        # TODO: These should be in a config file
        maxTrans = 1.5
        maxRot = 2 * math.pi

        LINEAR_RATE = math.pi / 2  # [rad/s]
        # WHEEL_SIZE = 0.195  # [m] From the manual
        WHEEL_SIZE = 0.1918986  # [m] From Webots Definition
        # BASE_SIZE = 0.3810  # [m] From the manual
        BASE_SIZE = 0.33  # [m] From WeBots Definition
        WHEEL_ROTATION = BASE_SIZE / WHEEL_SIZE

        rotation = positions[0]
        linear = positions[1]

        if not isinstance(rotation, (int, float)):
            rospy.logerr("Non-numeric rotation in list, aborting moveBase")
            return _states['ABORTED']
        elif not isinstance(linear, (int, float)):
            rospy.logerr("Non-numeric translation in list, aborting moveBase")
            return _states['ABORTED']
        if abs(linear) > maxTrans:
            rospy.logerr(
                "Maximal relative translation step exceeded(max: %sm), "
                "aborting moveBase" % maxTrans)
            return _states['ABORTED']
        if abs(rotation) > maxRot:
            rospy.logerr(
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
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
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
        rospy.loginfo("%s: Navigating to %s",
                      self._action_name,
                      pose)

        handle = _ActionHandle(client)
        client.send_goal(client_goal)
        handle.wait()
        return handle.result

    def moveJoints(self, goal, positions):
        if goal.component == 'tray':
            return self.moveTray(positions)

        return _states['SUCCEEDED']


# ------------------- action_handle section ------------------- #
# Action handle class.
#
# The action handle is used to implement asynchronous behaviour within the
# script.
class _ActionHandle(object):
        # Initialises the action handle.

    def __init__(self, simpleActionClient):
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
    rospy.init_node('sf_controller')
    sf = Sunflower(rospy.get_name())
    sf.run()
