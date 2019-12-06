#!/usr/bin/env python
import rospy
import rosparam
import numpy as np
from math import pi
import math
import tf
import cv2
import ipdb 
import geometry_msgs.msg
from std_msgs.msg      import String
from std_msgs.msg      import UInt16
from std_msgs.msg      import Int16
from utils import target_sub
# from utils import dolly_sub as DollyFB
from utils import dolly
from utils import arm

class arm_process():
    def __init__(self):
        self.fail_counter = {'NO_TOMATO':0,'C_FAILED':0,'SUCCEED':1,'T_FAILED':0}
    def init_counter(self):
        self.fail_counter['NO_TOMATO'] = 0
        self.fail_counter['C_FAILED']  = 0
        self.fail_counter['SUCCEED']   = 0
        self.fail_counter['T_FAILED']  = 0
        # self.fail_counter['OTW']       = 0
        # self.fail_counter['WAIT']      = 0


if __name__ == '__main__': 
    LorR = 'L'
    ######################################
    ## init_node & create arm commander
    ######################################
    rospy.init_node('main_process',anonymous=True)

    ################################
    ##instance to check the arm status
    ################################
    LarmProcess = arm_process()

    ##################################
    ###create handeye detect instance
    ##################################
    image_width  = int(rospy.get_param('/handeyeL/image_width'))
    image_height = int(rospy.get_param('/handeyeL/image_height'))

    conbeL = arm.Manipulator(LorR='L', handeye_w = image_width, handeye_h = image_height)
    conbeL.go_to_ready()

    ###############################
    ##Define the first move direction
    # MoveDirection : L => robot have to start from Right side
    # MoveDirection : R => robot have to start from Left side
    ###############################
    rane_dist = 1.2
    moveDirection = 'R'

    if(moveDirection == 'R'):
        left_thred = -0.01
        right_thred = rane_dist - 0.15
    else:
        left_thred = -rane_dist + 0.15
        right_thred = 0.01

    Dolly = dolly.Dolly_control(rane_dist,moveDirection)    
    Dolly.set_dist(0.28)
    ##################################


    while not rospy.is_shutdown():
        try:

            print('start main loop')
            stateL  = conbeL.main()
            print('state: ',stateL)
            # ipdb.set_trace()
            conbeL.go_to_ready() #arm l process
            continue

            #move dolly
            print('fail_counter: ',LarmProcess.fail_counter)
            # ipdb.set_trace()
             
            print('usual control')
            Dolly.move_dolly()
            # ipdb.set_trace()
            
            LarmProcess.init_counter()

            for i in range(2):
                stateL  = conbeL.main()
                print('state: ',stateL)
                ipdb.set_trace()
                LarmProcess.fail_counter[stateL] += 1
                print(LarmProcess.fail_counter)
                ipdb.set_trace()
                if(stateL == 'SUCCEED'):
                    break
                if(stateL == 'NO_TOMATO'):
                    break
            ipdb.set_trace()

            if(LarmProcess.fail_counter['SUCCEED'] == 0):
                print('distance control')
                Dolly.move_dolly_distance()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
