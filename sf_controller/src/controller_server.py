#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
import roslib
roslib.load_manifest('sf_controller')

import time
import math
import thread

import rospy
import actionlib
import sf_controller.msg, sf_lights.msg
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
from p2os_driver.msg import MotorState
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class SunflowerAction(object):

    _actionHandles = {}
    _cancelledComponents = {}

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, sf_controller.msg.SunflowerAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Started Sunflower Controller ActionServer")
        self._pubs = self._connectToJoints()
        self._subs = {}
        (self._cmdVel, self._motorState) = self._connectToWheels()
        self._feedback = sf_controller.msg.SunflowerFeedback()
        self._result = sf_controller.msg.SunflowerResult()        
                
        self._sfLight = actionlib.SimpleActionClient('/lights', sf_lights.msg.LightsAction)
        print "Waiting for sf_lights..."
        self._sfLight.wait_for_server()
        print "Connected to sf_lights"

    def __del__(self):
        for pub in self._pubs:
            pub.unregister()

    def _connectToWheels(self):
        return (rospy.Publisher('cmd_vel', Twist), rospy.Publisher('cmd_motor_state', MotorState))

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
        if goal.component == 'light':
            result = self.setlight(goal.jointPositions)
        if goal.action == 'move':
            result = self.move(goal)
        elif goal.action == 'init':
            result = self.init(goal.component)
        elif goal.action == 'stop':
            self.stop(goal.component)
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
        if name == 'base' or name == 'base_direct':
            SunflowerAction._cancelledComponents['base'] = True
            SunflowerAction._cancelledComponents['base_direct'] = True
        else:
            SunflowerAction._cancelledComponents[name] = True

        self._as.set_succeeded(self._result)

    def setlight(self, color):
        goal = sf_lights.msg.LightsGoal(rgb=color)
        return self._sfLight.send_goal_and_wait(goal)

    def move(self, goal):
        success = True
        joints = goal.jointPositions

        if(goal.namedPosition != '' and goal.namedPosition != None):
            param = '/sf_controller/' + goal.component + '/' + goal.namedPosition
            if(rospy.has_param(param)):
                joints = rospy.get_param(param)[0]

        rospy.loginfo("%s: Setting %s to %s",
                self._action_name,
                goal.component,
                goal.namedPosition or joints)

        if goal.component == 'base':
            success = self.navigate(goal, joints)
        elif goal.component == 'base_direct':
            success = self.moveBase(goal, joints)
        else:
            success = self.moveJoints(goal, joints)
            
        rospy.loginfo("%s: '%s to %s' succeeded:%s",
                self._action_name,
                goal.component,
                goal.namedPosition or joints,
                success)

        return success

    def moveBase(self, goal, positions):
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
                rospy.logerr("Maximal relative translation step exceeded(max: %sm), aborting moveBase" % maxTrans )
                return False
        if abs(rotation) >= maxRot:
                rospy.logerr("Maximal relative rotation step exceeded(max: %srad), aborting moveBase" % maxRot )
                return False
        
        # step 1: determine duration of motion so that upper thresholds for both translational as well as rotational velocity are not exceeded
        duration_trans_sec = abs(linear) / LINEAR_RATE
        duration_rot_sec = abs(rotation) / ROTATION_RATE
        duration_sec = max(duration_trans_sec, duration_rot_sec)
        duration_ros = rospy.Duration.from_sec(duration_sec)  # duration of motion in ROS time
        
        # step 2: determine actual velocities based on calculated duration
        x_vel = linear / duration_sec
        rot_vel = rotation / duration_sec
        
        # step 3: send constant velocity command to base_controller for the calculated duration of motion
#         pub = rospy.Publisher('cmd_vel', Twist)
        twist = Twist()
        twist.linear.x = x_vel
        twist.angular.z = rot_vel
        r = rospy.Rate(10)  # send velocity commands at 10 Hz
        SunflowerAction._cancelledComponents[goal.component] = False
        end_time = rospy.Time.now() + duration_ros
        self._cmdVel.publish(twist)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time and not SunflowerAction._cancelledComponents[goal.component]:
            #pub.publish(twist) #p2os has issues if you republish, seems to continue using last received cmd_vel
            r.sleep()
        
        self._cmdVel.publish(Twist()) #send a stop command, see above comment
    
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

        client.send_goal(client_goal)
        r = rospy.Rate(10)
        SunflowerAction._cancelledComponents[goal.component] = False
        while not rospy.is_shutdown() and not SunflowerAction._cancelledComponents[goal.component] and client.get_state() in ['ACTIVE', 'PENDING']:
            r.sleep()

        return client.get_result()

    def moveJoints(self, goal, positions):
        subs = []

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
            self._pubs[topic].publish(Float64(positions[i]))

        # TODO: Timeouts
        SunflowerAction._cancelledComponents[goal.component] = False
        reached = True
        while True:
            r = rospy.Rate(100)
            for sub in subs:
                if rospy.is_shutdown() or SunflowerAction._cancelledComponents[goal.component]:
                    return False

                while not sub.hasNewMessage:
                    r.sleep()

                if sub.lastMessage.is_moving:
                    continue
                
                reached &= abs(sub.lastMessage.error) < 0.01
                subs.remove(sub)
            
            if not subs:
                break                            

        return reached

#------------------- action_handle section -------------------#
# # Action handle class.
#
# The action handle is used to implement asynchronous behaviour within the script.
class _ActionHandle(object):
        # Initialises the action handle.
        def __init__(self, simpleActionClient):
            self._client = simpleActionClient

        @property
        def result(self):
            if self._waiting:
                return 1

            return self._result

        def wait(self, duration=None):
            self.wait_for_finished(duration, True)

        def waitAsync(self, duration=None):
            thread.start_new_thread(self.wait_for_finished, (duration, False,))

        def _wait_for_finished(self, duration):
            self._waiting = True
            if duration is None:
                self.client.wait_for_result()
            else:
                if not self.client.wait_for_result(rospy.Duration(duration)):
                    self._result = 9
                    return

            if self.client.get_state() != 3:
                self._result = 4
                return

            self._result = 3

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
