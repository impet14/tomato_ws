#! /usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import sys
import numpy as np
from jog_msgs.msg import JogFrame
import rospy
from sensor_msgs.msg import JointState
import tf

class jog_control():
    def __init__(self,frame_id,group_name,link_name,linear_delta,angular_delta):
        # Get parameters
        self.frame_id      = frame_id
        self.group_name    = group_name
        self.link_name     = link_name
        self.linear_delta  =  np.repeat(np.array([linear_delta]) ,3)
        self.angular_delta =  np.repeat(np.array([angular_delta]),3)
        self._linear_flag  = np.zeros(3)
        self._angular_flag = np.zeros(3)
        # Create jog frame command
        self._jog = JogFrame()
        self._jog.header.frame_id = self.frame_id
        self._jog.group_name      = self.group_name
        self._jog.link_name       = self.link_name
        # Publishers
        self._pub = rospy.Publisher('jog_frame', JogFrame, queue_size=10)
        self.r    = rospy.Rate(2)

    def set_linear_axis(self,x,y,z):
        self._linear_flag[0] = x
        self._linear_flag[1] = y
        self._linear_flag[2] = z
        self.lin_del    = self.linear_delta * self._linear_flag
        print(self.lin_del)
    
    def set_angular_axis(self,x,y,z):
        self._angular_flag[0] = x
        self._angular_flag[1] = y
        self._angular_flag[2] = z
        self.ang_del    = self.angular_delta * self._angular_flag
        print(self.ang_del)
    
    def move_with_linear_delta(self):
        '''jog a command with linear delta'''               
        self._jog.header.stamp = rospy.Time.now()
        self._jog.linear_delta.x = self.lin_del[0]
        self._jog.linear_delta.y = self.lin_del[1]
        self._jog.linear_delta.z = self.lin_del[2]
        self._pub.publish(self._jog)
        self.r.sleep()
        print(self._jog.linear_delta)

    def move_with_angular_delta(self):
        '''Test to jog a command with angular delta'''
        self._jog.header.stamp = rospy.Time.now()
        self._jog.angular_delta.x  = self.ang_del[0] / math.sqrt(3.0) 
        self._jog.angular_delta.y  = self.ang_del[1] / math.sqrt(3.0) 
        self._jog.angular_delta.z  = self.ang_del[2] / math.sqrt(3.0) 
        self._pub.publish(self._jog)
        self.r.sleep()
        print(self._jog.angular_delta)