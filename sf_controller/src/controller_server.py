#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
import roslib
roslib.load_manifest('sf_controller')

import time
import math

import rospy
import actionlib
import sf_controller.msg
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
from p2os_driver.msg import MotorState
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class SunflowerAction(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, sf_controller.msg.SunflowerAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Started Sunflower Controller ActionServer")
        self._pubs = self._connectToJoints()
        self._subs = {}
        self._motorState = self._connectToWheels()
        self._feedback = sf_controller.msg.SunflowerFeedback()
        self._result = sf_controller.msg.SunflowerResult()

    def __del__(self):
        for pub in self._pubs:
            pub.unregister()

    def _connectToWheels(self):
        return rospy.Publisher('cmd_motor_state', MotorState)

    def _connectToJoints(self):
        pubs = {}
        for value in rospy.get_param('/sf_controller').values():
            if 'joint_names' in value:
                for jointName in value['joint_names']:
                    topic = jointName + '_controller'
                    if topic not in pubs:
                        pubs[topic] = rospy.Publisher(topic + '/command', Float64)

        return pubs

    def execute_cb(self, goal):
        if goal.action == 'move':
            result = self.move(goal)
        elif goal.action == 'init':
            result = self.init(goal.component)
        else:
            rospy.logwarn("Unknown action %s", goal.action)
            self._result.result = -1
            self._as.set_aborted(self._result)

        if result:
            self._result.result = 0
            self._as.set_succeeded(self._result)

    def init(self, name):
        if name == 'base' or name == 'base_direct':
            self._motorState.publish(MotorState(1))
            self._as.set_succeeded(self._result)

    def move(self, goal):
        success = True
        joints = goal.jointPositions

        if(goal.namedPosition != '' and goal.namedPosition != None):
            param = '/sf_controller/' + goal.component + '/' + goal.namedPosition
            if(rospy.has_param(param)):
                joints = rospy.get_param(param)[0]

        rospy.loginfo("%s: Settings %s to %s (%s)",
                self._action_name,
                goal.component,
                goal.namedPosition,
                joints)

        if goal.component == 'base':
            success = self.navigate(goal, joints)
        elif goal.component == 'base_direct':
            success = self.moveBase(goal, joints)
        else:
            success = self.moveJoints(goal, joints)

        return success

    def moveBase(self, goal, positions):
        LINEAR_RATE = 0.5  # [m/s]
        ROTATION_RATE = 0.5  # [rad/s]

        rotation = positions[0]
        linear = positions[1]

        rospy.loginfo("Moving base %s rad and %s meters", rotation, linear)

        # step 0: check validity of parameters:
        if not isinstance(rotation, (int, float)) or not isinstance(linear, (int, float)):
                rospy.logerr("Non-numeric rotation list, aborting moveBase")
                return False
        if math.sqrt(linear ** 2) >= 0.15:
                rospy.logerr("Maximal relative translation step exceeded, aborting moveBase")
                return False
        if abs(rotation) >= math.pi / 2:
                rospy.logerr("Maximal relative rotation step exceeded, aborting moveBase")
                return False

        # step 1: determine duration of motion so that upper thresholds for both translational as well as rotational velocity are not exceeded

        duration_trans_sec = math.sqrt(linear ** 2) / LINEAR_RATE
        duration_rot_sec = abs(rotation / ROTATION_RATE)
        duration_sec = max(duration_trans_sec, duration_rot_sec)
        duration_ros = rospy.Duration.from_sec(duration_sec)  # duration of motion in ROS time

        # step 2: determine actual velocities based on calculated duration
        x_vel = linear / duration_sec
        rot_vel = rotation / duration_sec

        # step 3: send constant velocity command to base_controller for the calculated duration of motion
        pub = rospy.Publisher('cmd_vel', Twist)
        twist = Twist()
        twist.linear.x = x_vel
        twist.angular.z = rot_vel
        r = rospy.Rate(10)  # send velocity commands at 10 Hz
        end_time = rospy.Time.now() + duration_ros
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                pub.publish(twist)
                r.sleep()

        if pub:
            pub.unregister()
        return True

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
        return client.send_goal_and_wait(client_goal)

    def moveJoints(self, goal, positions):
        subs = []

        component_name = '/sf_controller/' + goal.component
        joint_names = rospy.get_param(component_name + '/joint_names')
        for i in range(0, len(joint_names)):
            topic = joint_names[i] + '_controller'
            if topic not in self._pubs:
                rospy.logerr('Undefined joint controller %s', topic)

            subs.append(RosSubscriber(topic + '/state', JointState))
            self._pubs[topic].publish(Float64(positions[i]))

        # TODO: Timeouts
        done = False
        while(not done):
            done = True
            for sub in subs:
                while not sub.hasNewMessage:
                    rospy.sleep(0.01)

                if(sub.lastMessage.goal_pos != 0):
                    reached = not sub.lastMessage.is_moving and abs(sub.lastMessage.current_pos - sub.lastMessage.goal_pos) / sub.lastMessage.goal_pos < .1
                elif(sub.lastMessage.current_pos != 0):
                    reached = not sub.lastMessage.is_moving and abs(sub.lastMessage.current_pos - sub.lastMessage.goal_pos) / sub.lastMessage.current_pos < .1
                else:
                    reached = True
                done = done and reached

        return done


#------------------- action_handle section -------------------#
# # Action handle class.
#
# The action handle is used to implement asynchronous behaviour within the script.
class _ActionHandle(object):
        # Initialises the action handle.
        def __init__(self, simpleActionClient):
            self._client = simpleActionClient

        def wait(self, duration=None):
            self.blocking = True
            self.wait_for_finished(duration, True)

        def wait_inside(self, duration=None):
            if self.blocking:
                self.wait_for_finished(duration, True)
            else:
                thread.start_new_thread(self.wait_for_finished, (duration, False,))
            return self.error_code

        def wait_for_finished(self, duration, logging):
            if duration is None:
                if logging:
                        rospy.loginfo("Wait for <<%s>> reaching <<%s>>...", self.component_name, self.parameter_name)
                self.client.wait_for_result()
            else:
                if logging:
                    rospy.loginfo("Wait for <<%s>> reached <<%s>> (max %f secs)...", self.component_name, self.parameter_name, duration)
                if not self.client.wait_for_result(rospy.Duration(duration)):
                    if logging:
                        rospy.logerr("Timeout while waiting for <<%s>> to reach <<%s>>. Continuing...", self.component_name, self.parameter_name)
                    self.set_failed(10)
                    return
            # check state of action server
            # print self.client.get_state()
            if self.client.get_state() != 3:
                if logging:
                    rospy.logerr("...<<%s>> could not reach <<%s>>, aborting...", self.component_name, self.parameter_name)
                self.set_failed(11)
                return

            if logging:
                rospy.loginfo("...<<%s>> reached <<%s>>", self.component_name, self.parameter_name)

            self.set_succeeded()  # full success

        # # Cancel action
        #
        # Cancels action goal(s).
        def cancel(self):
                self.client.cancel_all_goals()


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
        if self._subscriber == None:
            self._subscriber = rospy.Subscriber(self._topic, self._dataType, self._callback)

    def unregister(self):
        if self._subscriber != None:
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
