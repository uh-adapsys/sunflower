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
        self._motorState = self._connectToWheels()
        self._subs = {}
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
            if value.has_key('joint_names'):
                for jointName in value['joint_names']:
                    topic = jointName + '_controller'
                    if not pubs.has_key(topic):
                        pubs[topic] = rospy.Publisher(topic + '/command', Float64)
                
        return pubs
                    
        
    def execute_cb(self, goal):
        
        if goal.action == 'move':
            return self.move(goal)
        elif goal.action == 'init':
            return self.init(goal.component)
        else:
            rospy.logwarn("Unknown action %s", goal.action)
            
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
        
        if success:
            self._as.set_succeeded(self._result)
    
    def moveBase(self, goal, positions):
        LINEAR_RATE = 0.5
        ROTATION_RATE = 0.5
        
        rotation = positions[0]
        linear = positions[1]

        #Rounded to the hundredths of a second (the rate at which p2os communicates with the base)
        rotationTime = round(abs(rotation) / ROTATION_RATE, 1)
        linearTime = round(abs(linear) / LINEAR_RATE, 1)
        
        try:
            rospy.loginfo("Moving base %s rad and %s meters", rotation, linear)
            
            cmdVel = rospy.Publisher('cmd_vel', Twist)
            #first twist
            msg = Twist()
            msg.angular.z = math.copysign(ROTATION_RATE, rotation)
            for _ in range(0, rotationTime, 0.1):
                cmdVel.publish(msg)
                rospy.sleep(0.1)
            
            #now move
            msg = Twist()
            msg.linear.x = math.copysign(LINEAR_RATE, linear)
            for _ in range(0, linearTime, 0.1):
                cmdVel.publish(msg)
                rospy.sleep(0.1)
        except Exception as e:
            rospy.logerror("Error when moving base: %s", e)
            return False
        finally:
            if cmdVel != None:
                #full stop
                cmdVel.publish(Twist())
                cmdVel.unregister()
                
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
            if not self._pubs.has_key(topic):
                rospy.logerr('Undefined joint controller %s', topic)
            
            subs.append(RosSubscriber(topic + '/state', JointState))
            self._pubs[topic].publish(Float64(positions[i]))
            
            
        #TODO: Timeouts
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
