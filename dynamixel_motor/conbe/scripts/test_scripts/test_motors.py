#!/usr/bin/env python
import rospy
import numpy as np
from math import pi
import math
import tf
# import cv2
# import geometry_msgs.msg
# from std_msgs.msg      import String
# from std_msgs.msg      import UInt16
# from geometry_msgs.msg import Quaternion
# from cv_bridge         import CvBridge, CvBridgeError
# from visualization_msgs.msg       import Marker
##import custom class
from utils import dxl_move as DXL
from utils import conbe_ik as CONBE
from utils import client_trajectory

if __name__ == '__main__': 

    ##########################################
    ## create instancd to calculate IK
    ##########################################
    pos_bound = [0.05,0.05,0.05]
    ori_bound = [0.1,0.1,0.5]

    conbeL_ik   = CONBE.ConbeIK(urdf_param='/LArm/robot_description',LorR='L',pos_bound=pos_bound,ori_bound=ori_bound)
    conbeL_ik.check_setting()

    conbeR_ik   = CONBE.ConbeIK(urdf_param='/RArm/robot_description',LorR='R',pos_bound=pos_bound,ori_bound=ori_bound)
    conbeR_ik.check_setting()
    #########################################
    ## create instance to control the arm using FollowJointTrajectory
    #########################################
    conbeL_arm = client_trajectory.Joint('/LArm/conbeL_controller') 
    conbeR_arm = client_trajectory.Joint('/RArm/conbeR_controller') 

    print ('init angle')
    conbeL_arm.move([0.0,0.0,0.0,0.0,0.0,0.0])
    conbeR_arm.move([0.0,0.0,0.0,0.0,0.0,0.0])





