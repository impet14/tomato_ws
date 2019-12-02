#!/usr/bin/env python
import rospy
import rosparam
import numpy as np
from math import pi
import math
import tf
import cv2
import geometry_msgs.msg
from std_msgs.msg      import String
from std_msgs.msg      import Int16

class Arm_state_commander():
    def __init__(self,LorR):
        self.arm = LorR + 'Arm'
        self.pub_dist = '/' + self.arm + '/main_command'
        self.sub_dist = '/' + self.arm + '/movement_status'
        self.pub = rospy.Publisher(self.pub_dist,Int16,queue_size=1)
        self.sub =  rospy.Subscriber(self.sub_dist, Int16, self.callback,queue_size=1)
        self._arm_response_dict = {-1:'NO_TOMATO',0:'C_FAILED',1:'SUCCEED',5:'T_FAILDED',10:'OTW',50:'WAIT'}
        self._arm_response = 'WAIT'
        self._prev_response = 'None'
    def start_moving(self):
        for i in range(3):
            self.pub.publish(Int16(1))
    def stop_moving(self):
        for i in range(3):
            self.pub.publish(Int16(0))

    def get_arm_status(self):
        return self._arm_response
    
    def msg_interpreter(self,msg):
        self._arm_response = self._arm_response_dict[msg]
        if(self._arm_response != self._prev_response):
            print(self.arm + ' : ' + self._arm_response)
            self._prev_response = self._arm_response
    
    def callback(self,msg):
        self.msg_interpreter(msg.data)

if __name__ == '__main__':
    rospy.init_node('Arm_main',anonymous=True)
    LArm = Arm_state_commander('L')

    while not rospy.is_shutdown():
        try:
            print('start main loop')

            for i in range(5):
                LArm.start_moving()
                rospy.sleep(1)  
                LArm.stop_moving()
                rospy.sleep(1)  
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue