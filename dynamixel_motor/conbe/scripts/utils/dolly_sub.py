#!/usr/bin/env python
import rospy
import rosparam
import numpy as np
from math import pi
import math
import tf
import cv2
import geometry_msgs.msg
from std_msgs.msg   import Float64

class Dolly_feedback():
    def __init__(self):
        self._sub_dist = '/Dolly/state'
        self._dist = 0.0  ##tatal distance
        self._local_offset = 0.0
        self._offset = 0
        self._sub =  rospy.Subscriber(self._sub_dist, Float64, self.callback,queue_size=1)

    #set current distance as a offset
    def set_offset(self):
        self._local_offset = self._dist
    
    #############################
    ## to get local distance , 
    ## reference from offset
    ## and get local_state()
    #############################
    def get_local_state(self):
        print('dist         : ',self._dist)
        print('local_offset : ',self._local_offset)
        return self.calibrated_value(self._dist - self._local_offset)
        
    def get_state(self):
        return self.calibrated_value(self._dist)
    
    def calibrated_value(self,dist):
        #calibrate actual fb usung bias by experimental results
        return dist * 1.212
        
    def callback(self,msg):
        if(self._offset == 0):
            ###use first feedback as a offset
            self._offset = self._dist
        self._dist = msg.data - self._offset