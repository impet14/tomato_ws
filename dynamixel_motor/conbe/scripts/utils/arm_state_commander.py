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
        self.arm_interpret_dict = {-1:'NO_TOMATO',-2:'C_FAILED',2:'SUCCEED',5:'T_FAILED',1:'OTW',0:'WAIT'}
        self.msg_confirmation = 10000
        self.arm_response = 'WAIT'
        self.prev_response = 'None'

    def start_moving(self):
        self.publish_command_until_get_response(1)
        
    def stop_moving(self):
        self.publish_command_until_get_response(0)
            
    def publish_command_until_get_response(self,command): ##command {0 or 1}
        while(command != self.msg_confirmation):
            self.pub.publish(Int16(command))
            rospy.sleep(0.01)
    
    def msg_interpreter(self,msg):
        self.arm_response = self.arm_interpret_dict[msg]
        if(self.arm_response != self.prev_response):
            print('arm_status-FB.....',self.arm + ' : ' + self.arm_response)
            self.prev_response = self.arm_response
    
    def callback(self,msg):
        self.msg_interpreter(msg.data)
        #message for confirmation
        self.msg_confirmation = msg.data
        if(mag.data != 0 and msg.data != 1):
            ##when receive the msg, resend the same value to confirm that 
            ##this node successfuly receive the msg
            self.pub.publish(msg.data)


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