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
from geometry_msgs.msg import Quaternion
from cv_bridge         import CvBridge, CvBridgeError
from visualization_msgs.msg       import Marker
from utils import dxl_move as DXL
from utils import handeye_sub as HandEye
from utils import conbe_ik as CONBE
from utils import client_trajectory
from utils import arm_state_commander as arm_master


def dolly_mode_interpreter(command):
    if(command == 49):
        print('stop')
    elif(command < 49):
        print('left')
    else:
        print('right')
def key_control():
    background = np.zeros((200,300,3), np.uint8)
    cv2.namedWindow('input_test')

    flag = True
    while(flag):
        cv2.imshow('input_test',background)
        input = cv2.waitKey(10)
        if input == ord('r'):  # if key 'z' is pressed 
            print('r-pressed: Go to right')
            dolly_pub.publish(80)
        elif input == ord('l'):  # if key 'x' is pressed 
            print('l-pressed: Go to left')
            dolly_pub.publish(20)
        elif input == ord('q'): # break
            cv2.destroyAllWindows()
            break
        elif input == ord('s'):
            print('stop')
            dolly_pub.publish(49)



if __name__ == '__main__': 
    ## init_node & create arm commander
    rospy.init_node('dolly_test',anonymous=True)

    ## DORY control msg
    ##msg can be interpret as below
    # HERE, use dolly_mode_interpreter func 
    # start-R : 50~99
    # start-L  : 0~48
    # stop         : 49
    dolly_pub = rospy.Publisher("dolly", UInt16, queue_size = 1)

 

    while not rospy.is_shutdown():
        try:
            key_control()
            continue

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
