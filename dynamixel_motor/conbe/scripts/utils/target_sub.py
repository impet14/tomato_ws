#!/usr/bin/env python
import rospy
import rosparam
import numpy as np
from math import pi
import math
import tf
import cv2
import geometry_msgs.msg
from easy_markers.generator import *
from visualization_msgs.msg import Marker

class target_subscriber():
    def __init__(self,LorR):
        self._LorR  = LorR
        self._sub_dist = "/target_marker_" + self._LorR + "link0_frame"
        self.init_marker()
        self._sub =  rospy.Subscriber(self._sub_dist, Marker, self.callback,queue_size=1)

    def init_marker(self):
        #############################################
        #Marker Publisher Initialization
        #############################################
        hand_mark = MarkerGenerator()
        hand_mark.type = Marker.SPHERE_LIST
        hand_mark.scale = [.03]*3
        hand_mark.frame_id = '/' + self._LorR + 'link0'
        #target mark
        hand_mark.counter = 0
        t = rospy.get_time()
        hand_mark.color = [0,1,0,1]
        self._msg = hand_mark.marker(points= [(-1, -1, -1)])        

    def getXYZ(self):
        x = self._msg.points[0].x
        y = self._msg.points[0].y
        z = self._msg.points[0].z
        return (x,y,z)

    def get(self):
        return self._msg
        
    def callback(self,msg):
        # print(msg)
        self._msg = msg