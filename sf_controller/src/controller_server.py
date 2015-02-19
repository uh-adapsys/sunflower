#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
from threading import Thread
import math
import time

import roslib
roslib.load_manifest('sf_controller')

from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from p2os_driver.msg import MotorState
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler
import actionlib
import rospy
import sf_controller_msgs.msg
import sf_lights_msgs.msg


class SunflowerAction(object):

    _actionHandles = {}

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, sf_controller_msgs.msg.SunflowerAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Started Sunflower Controller ActionServer")
        self._pubs = self._connectToJoints()
        self._subs = {}
        (self._cmdVel, self._motorState) = self._connectToWheels()
        self._feedback = sf_controller_msgs.msg.SunflowerFeedback()
        self._result = sf_controller_msgs.msg.SunflowerResult()

    def __del__(self):
        for pub in self._pubs:
            pub.unregister()

    def park(self):
        g = sf_controller_msgs.msg.SunflowerAction()
        g.component = 'head'
        g.positions = [0, 0, -1.67, 1.67]
        self.execute_cb(g)
        g = sf_controller_msgs.msg.SunflowerAction()
        g.component = 'tray'
        g.positions = [0, ]
        self.execute_cb(g)
        g = sf_controller_msgs.msg.SunflowerAction()
        g.component = 'lights'
        g.positions = [0, 0, 0]
        self.execute_cb(g)

    def _connectToWheels(self):
        return (rospy.Publisher('cmd_vel', Twist), rospy.Publisher('cmd_motor_state', MotorState))

    def _connectToJoints(self):
        pubs = {}
        joints = rospy.get_param('/sf_controller', None)
        if joints:
            for value in joints.values():
                if 'joint_names' in value:
                    for jointName in value['joint_names']:
                        topic = jointName + '_controller'
                        if topic not in pubs:
                            pubs[topic] = rospy.Publisher(
                                topic + '/command', Float64)
        else:
            rospy.logerr("%s:  No joints received from parameter server",
                         self._action_name)

        return pubs

    def execute_cb(self, goal):
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
            self._motorState.publish(MotorState(1))

        self._as.set_succeeded(self._result)

    def stop(self, name):
        rospy.loginfo("%s: Stopping %s",
                      self._action_name,
                      name)
        if name == 'base':
            client = actionlib.SimpleActionClient(
                '/lights', sf_lights_msgs.msg.LightsAction)
            client.wait_for_server()
            client.cancel_all_goals()
        elif name == 'light':
            client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            client.wait_for_server()
            client.cancel_all_goals()
        else:
            pass

        self._as.set_succeeded(self._result)

    def setlight(self, color):
        client = actionlib.SimpleActionClient(
            '/lights', sf_lights_msgs.msg.LightsAction)
        client.wait_for_server()
        goal = sf_lights_msgs.msg.LightsGoal(rgb=color)
        handle = _ActionHandle(client)
        client.send_goal(goal)
        handle.wait()
        handle.result

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

        return result == 3

    def moveBase(self, goal, positions):
        # TODO: These should be in a config file
        maxTrans = 1.5
        LINEAR_RATE = 0.3  # [m/s]
        maxRot = 2 * math.pi
        ROTATION_RATE = 0.5  # [rad/s]

        rotation = positions[0]
        linear = positions[1]

        rospy.loginfo("Moving base %s rad and %s meters", rotation, linear)

        # step 0: check validity of parameters:
        if not isinstance(rotation, (int, float)) or not isinstance(linear, (int, float)):
            rospy.logerr("Non-numeric rotation list, aborting moveBase")
            return False
        if abs(linear) >= maxTrans:
            rospy.logerr(
                "Maximal relative translation step exceeded(max: %sm), aborting moveBase" % maxTrans)
            return False
        if abs(rotation) >= maxRot:
            rospy.logerr(
                "Maximal relative rotation step exceeded(max: %srad), aborting moveBase" % maxRot)
            return False

        # step 1: determine duration of motion so that upper thresholds for
        # both translational as well as rotational velocity are not exceeded
        duration_trans_sec = abs(linear) / LINEAR_RATE
        duration_rot_sec = abs(rotation) / ROTATION_RATE
        duration_sec = max(duration_trans_sec, duration_rot_sec)

        # step 2: determine actual velocities based on calculated duration
        x_vel = linear / duration_sec
        rot_vel = rotation / duration_sec

        # step 3: send constant velocity command to base_controller for the calculated duration of motion
        # pub = rospy.Publisher('cmd_vel', Twist)
        twist = Twist()
        twist.linear.x = x_vel
        twist.angular.z = rot_vel
        # duration of motion in ROS time
        duration_ros = rospy.Duration.from_sec(duration_sec)

        r = rospy.Rate(50)
        end_time = rospy.Time.now() + duration_ros
        self._cmdVel.publish(twist)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            # pub.publish(twist) #p2os has issues if you republish, seems to
            # continue using last received cmd_vel
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                return 2
            r.sleep()

        self._cmdVel.publish(Twist())  # send a stop command, see above comment

        return 3

    def navigate(self, goal, positions):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
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
        subs = []
        pubs = {}

        component_name = '/sf_controller/' + goal.component
        try:
            joint_names = rospy.get_param(component_name + '/joint_names')
        except KeyError:
            # assume component is a named joint
            joint_names = [goal.component, ]
        for i in range(0, len(joint_names)):
            topic = joint_names[i] + '_controller'
            if topic not in self._pubs:
                rospy.logerr('Undefined joint controller %s', topic)
                return False

            subs.append(RosSubscriber(topic + '/state', JointState))
            pubs[topic] = self._pubs[topic]
            self._pubs[topic].publish(Float64(positions[i]))

        handle = _SubscriberHandle(subs, pubs)
        handle.wait()
        return handle.result


# ------------------- action_handle section ------------------- #
class _SubscriberHandle(object):
    # Subscriber handle class.
    #
    # The subscriber handle is used to implement asynchronous behaviour within
    # the script.

    def __init__(self, subscribers, publishers):
        # Initialises the action handle.
        self._subscribers = subscribers
        self._publishers = publishers
        self._waiting = False
        self._result = None

    @property
    def result(self):
        if self._waiting:
            return None

        return self._result

    def wait(self, duration=None):
        t = self.waitAsync(duration)
        # An attempt to fix parallel action issue
        # t.join()
        # Potential race condition, isAlive returns false before starting
        time.sleep(0.01)
        while t.isAlive():
            time.sleep(0.001)

    def waitAsync(self, duration=None):
        thread = Thread(target=self._wait_for_finished, args=(duration,))
        thread.setDaemon(True)
        thread.start()
        return thread

    def _wait_for_finished(self, duration):
        self._waiting = True
        reached = True
        while True:
            r = rospy.Rate(100)
            for sub in self._subscribers:
                if rospy.is_shutdown():
                    return False

                while not sub.hasNewMessage:
                    r.sleep()

                if sub.lastMessage.is_moving:
                    continue

                reached &= abs(sub.lastMessage.error) < 0.03
                if not reached:
                    rospy.logwarn("%s unable to reach %s.  Error value: %s" % (
                        sub.topic, sub.lastMessage.goal_pos, sub.lastMessage.error))

                self._subscribers.remove(sub)

            if not self._subscribers:
                break

        self._result = 3 if reached else 4

    def cancel(self):
        for sub in self._subscribers:
            self._publishers[sub.topic].publish(
                Float64(sub.lastMessage.current_pos))


# ------------------- action_handle section ------------------- #
class _ActionHandle(object):
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
        # An attempt to fix parallel action issue
        # t.join()
        # Potential race condition, isAlive returns false before starting
        time.sleep(0.01)
        while t.isAlive():
            time.sleep(0.001)

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


class RosSubscriber(object):

    def __init__(self, topic, dataType, idleTime=15):
        self._lastAccess = time.time()
        self._subscriber = None
        self._topic = topic
        self._dataType = dataType
        self._newMessage = False
        self._idleTimeout = idleTime
        self._data = None

    @property
    def topic(self):
        return self._topic

    @property
    def hasNewMessage(self):
        self._touch()
        return self._newMessage

    @property
    def lastMessage(self):
        self._touch()
        self._newMessage = False
        return self._data

    def _touch(self):
        self._lastAccess = time.time()
        if self._subscriber is None:
            self._subscriber = rospy.Subscriber(
                self._topic, self._dataType, self._callback)

    def unregister(self):
        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None

    def _callback(self, msg):
        self._data = msg

        self._newMessage = True
        if time.time() - self._lastAccess > self._idleTimeout:
            self.unregister()

if __name__ == '__main__':

    rospy.init_node('sf_controller')
    SunflowerAction(rospy.get_name())
    rospy.spin()
