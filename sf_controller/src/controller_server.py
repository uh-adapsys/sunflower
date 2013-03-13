#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
import roslib
roslib.load_manifest('sf_controller')

import time
import rospy
import actionlib
import sf_controller.msg
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
from threading import Thread

class SunflowerAction(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, sf_controller.msg.SunflowerAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Started Sunflower Controller ActionServer")
        self._pubs = {}
        self._subs = {}
        self._feedback = sf_controller.msg.SunflowerFeedback()
        self._result = sf_controller.msg.SunflowerResult()

        
    def __del__(self):
        for pub in self._pubs:
            pub.unregister()
        
    def execute_cb(self, goal):
        success = True
        rospy.loginfo("%s: Settings %s to %s",
                self._action_name,
                goal.component,
                goal.namedPosition or goal.jointPositions)
        
        component_name = '/sf_controller/' + goal.component + '/'
        joint_names = rospy.get_param(component_name + '/joint_names')
        joints = goal.jointPositions
        
        if(goal.namedPosition != '' and goal.namedPosition != None):
            param = component_name + '/' + goal.namedPosition
            if(rospy.has_param(param)):
                joints = rospy.get_param(param)[0]
        
        subs = []
        
        for i in range(0, len(joint_names)):
            topic = joint_names[i] + '_controller'
            if not self._pubs.has_key(topic):
                self._pubs[topic] = RosPublisher(topic + '/command', Float64)
                self._pubs[topic].start()
            
            subs.append(RosSubscriber(topic + '/state', JointState))
            self._pubs[topic].publish(Float64(joints[i]))
            
            
        #TODO: Timeouts
        done = False
        while(not done):
            done = True
            for sub in subs:
                while not sub.hasNewMessage:
                    time.sleep(0.01)
                
                reached = not sub.lastMessage.is_moving and abs(sub.lastMessage.current_pos - sub.lastMessage.goal_pos) < 10
                done = done and reached
        
        if success:
            self._result = 0
            self._as.set_succeeded(self._result)
            
            
class RosPublisher(Thread):
    
    def __init__(self, topic, dataType):
        self._topic = topic
        self._dataType = dataType;
        self._newMsg = None
        self._cancelRequested = False
        super(RosPublisher, self).__init__()
        
    def publish(self, msg):
        self._newMsg = msg
        
    def unsubscribe(self):
        self._cancelRequested = True
        if self._pub != None:
            self._pub.unsubscribe()
        
    def run(self):
        self._pub = rospy.Publisher(self._topic, self._dataType)
        
        while not self._cancelRequested:
            if self._newMsg != None:
                msg = self._newMsg
                self._newMsg = None
                self._pub.publish(msg)
            else:
                time.sleep(0.01)
        

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
