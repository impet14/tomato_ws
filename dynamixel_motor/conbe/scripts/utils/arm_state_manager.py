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
from std_msgs.msg      import UInt16

class Arm_state_manager():
    def __init__(self,LorR):
        self.arm = LorR + 'Arm'
        self.pub_dist = '/' + self.arm + '/movement_status'
        self.sub_dist = '/' + self.arm + '/main_command'
        self.pub =  rospy.Publisher(self.pub_dist, UInt16,queue_size=1)
        self.sub = rospy.Subscriber(self.sub_dist,UInt16,self.callback,queue_size=1)
        self._command = False
        self.arm_msg_dict = {0:False,1:True}
        self._arm_response_dict = {'C_FAILED':0,'SUCCEED':1,'T_FAILDED':5,'OTW':10,'WAIT':50}
    def publish_state(self,state):
        for i in range(3):
            self.pub.publish(UInt16(self._arm_response_dict[state]))

    def get_command(self):
        return self._command

    def msg_interpreter(self,msg):
        # here get the command fro main
        self._command = self.arm_msg_dict[msg]
    
    def callback(self,msg):
        print(msg.data)
        self.msg_interpreter(msg.data)



if __name__ == '__main__':
    ############CHANGE HERE up to the arm###
    rospy.init_node('Arm_interface', anonymous=True)
    LorR = 'L'
    Arm = Arm_state_manager(LorR)

    while not rospy.is_shutdown():
        print('start ' + LorR  + 'Arm loop')
        for i in range(5):
            Arm.publish_state('C_FAILED')
            print('command from main: ',Arm.get_command())
            rospy.sleep(0.5)        
            Arm.publish_state('SUCCEED')
            print('command from main: ',Arm.get_command())
            rospy.sleep(0.5)        
            Arm.publish_state('T_FAILDED')
            print('command from main: ',Arm.get_command())
            rospy.sleep(0.5)        
            Arm.publish_state('OTW')
            print('command from main: ',Arm.get_command())
            rospy.sleep(0.5)        
            Arm.publish_state('WAIT')
            print('command from main: ',Arm.get_command())
            rospy.sleep(0.5)        
        break
    