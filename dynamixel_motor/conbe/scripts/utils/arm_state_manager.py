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

class Arm_state_manager():
    def __init__(self,LorR):
        self.arm = LorR + 'Arm'
        self.pub_dist = '/' + self.arm + '/movement_status'
        self.sub_dist = '/' + self.arm + '/main_command'
        self.pub =  rospy.Publisher(self.pub_dist, Int16,queue_size=1)
        self.sub = rospy.Subscriber(self.sub_dist,Int16,self.callback,queue_size=1)
        self.state = 50 #WAIT
        self.command = False
        self.msg_confirmation = 10000
        self.arm_msg_dict = {0:'WAIT',1:'OTW'}
        self.arm_response_dict = {'NO_TOMATO':-1,'C_FAILED':-2,'SUCCEED':2,'T_FAILED':5,'OTW':1,'WAIT':0}
    
    def set_state(self,state):
        #keep arm state in variable (Int)
        self.state = state
    
    def publish_current_state(self):
        self.pub.publish(Int16(self.arm_response_dict[self.state]))

    def publish_state(self,state):
        #update the current state
        self.set_state(state)
        #publish state
        self.publish_current_state()
    
    def publish_state_until_get_response(self,state):
        if(state == 'WAIT' or state == 'OTW'):
            print('cannot send this state using this function to in arm_state_manager')
        else:
            while(self.arm_response_dict[state] != self.msg_confirmation):
                self.publish_state(state)
                rospy.sleep(0.01)

    def callback(self,msg):
        #message for command
        if(msg.data == 0 or msg.data == 1):
            print(self.arm_msg_dict[msg.data])
            # recend the msg
            self.publish_state(self.arm_msg_dict[msg.data])
        # save the latest received msg
        self.msg_confirmation = msg.data



if __name__ == '__main__':
    ############CHANGE HERE up to the arm###
    rospy.init_node('Arm_interface', anonymous=True)
    LorR = 'L'
    Arm = Arm_state_manager(LorR)

    while not rospy.is_shutdown():
        print('start ' + LorR  + 'Arm loop')
        for i in range(5):
            Arm.publish_state('C_FAILED')
            print('command from main: ',Arm.command())
            rospy.sleep(0.5)        
            Arm.publish_state('SUCCEED')
            print('command from main: ',Arm.command())
            rospy.sleep(0.5)        
            Arm.publish_state('T_FAILDED')
            print('command from main: ',Arm.command())
            rospy.sleep(0.5)        
            Arm.publish_state('OTW')
            print('command from main: ',Arm.command())
            rospy.sleep(0.5)        
            Arm.publish_state('WAIT')
            print('command from main: ',Arm.command())
            rospy.sleep(0.5)        
        break
    