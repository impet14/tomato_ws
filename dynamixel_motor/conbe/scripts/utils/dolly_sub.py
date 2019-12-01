#!/usr/bin/env python
import rospy
import rosparam
import numpy as np
from math import pi
import math
import tf
import cv2
import geometry_msgs.msg
from std_msgs.msg   import Float32

class Arm_state_commander():
    def __init__(self,LorR):
        self._sub_dist = '/Dolly/state'
        self._dist = 0.0
        self._sub =  rospy.Subscriber(self.sub_dist, Float32, self.callback,queue_size=1)

    def get_dolly_state(self):
        return self._dist
        
    def callback(self,msg):
        self._dist = msg.data