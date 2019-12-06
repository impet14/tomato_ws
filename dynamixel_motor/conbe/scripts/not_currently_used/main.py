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
from geometry_msgs.msg import Quaternion
from cv_bridge         import CvBridge, CvBridgeError
from visualization_msgs.msg       import Marker
from utils import dxl_move as DXL
from utils import handeye_sub as HandEye
from utils import conbe_ik as CONBE
from utils import client_trajectory
from utils import arm_state_commander as arm_master
from utils import target_sub
from utils import dolly_sub as DollyFB


# def create_marker(axis):
#     marker = Marker()
#     marker.header.frame_id = 'Llink0'
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "visulize-eef-orientation"
#     marker.id = 0 if(axis=='x') else 1 if(axis=='y')  else 2
#     marker.action = Marker.ADD
#     marker.color.r = 1.0 if(axis=='x') else 0.0
#     marker.color.g = 1.0 if(axis=='y') else 0.0
#     marker.color.b = 1.0 if(axis=='z') else 0.0
#     marker.color.a = 1.0 
#     marker.scale.x = 0.1
#     marker.scale.y = 0.01
#     marker.scale.z = 0.01
#     return marker

# def set_marker_param(marker,axis,px,py,pz,quaternion):
#     ##offest the orientation
#     ## conversion to represent each axis. since arrow marker is along with x-axis as a default,
#     ## this is the conversion quaternion 
#     orientation =  {'x' :[0.0,0.0,0.0,1],
#                     'y' :[0.0,0.0,0.7071068,0.707168],
#                     'z' :[0.0,-0.7071068,0.0,0.7071068]}
#     ###marker to visualize eef arrow
#     marker.pose.position.x = px
#     marker.pose.position.y = py
#     marker.pose.position.z = pz
#     #since the original arrow is along with x-axis,need to offset make it same as link0 frame.
#     #after this, put the one
#     link0_to_EEF = np.asarray(orientation[axis], dtype=np.float32)
#     q_new = tf.transformations.quaternion_multiply(quaternion, link0_to_EEF)
#     marker.pose.orientation.x=q_new[0]
#     marker.pose.orientation.y=q_new[1]
#     marker.pose.orientation.z=q_new[2]
#     marker.pose.orientation.w=q_new[3]
#     marker.lifetime = rospy.Duration()
#     marker.type = 0
#     return marker

# def set_pre_position(q_orientation,px,py,pz,L):
#     ##convert to matrix
#     matrix = tf.transformations.quaternion_matrix(q_orientation)
#     # print('matrix: ',matrix)
#     ##set the point
#     point = np.matrix([0, 0, L, 1], dtype='float32')
#     point.resize((4, 1))
#     ##get the point along with eef_frame
#     rotated = matrix*point
#     ##get the pre-grip position
#     pre_position = np.array([ px + rotated.item(0), py + rotated.item(1), pz + rotated.item(2)])
#     return pre_position
    
# def calc_orientation(px,py,pz,offset_z):
#     yaw= math.atan2(py,px)
#     z_offset = math.fabs(pz-offset_z)
#     Beta = math.atan2(z_offset,math.sqrt(px*px+py*py))
#     pitch = Beta - pi/2 if(pz-offset_z > 0) else - Beta - pi/2
#     return pitch,yaw


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

    while not rospy.is_shutdown():
        try: 
            cv2.imshow('input_test',background)
            print('Reference distance:  ',dolly_sub.get_state())
            input = cv2.waitKey(0)
            if input == ord('r'):  # if key 'z' is pressed 
                print('r-pressed: Go to right')
                dolly_pub.publish(59)
            elif input == ord('l'):  # if key 'x' is pressed 
                print('l-pressed: Go to left')
                dolly_pub.publish(39)
            elif input == ord('q'): # break
                cv2.destroyAllWindows()
                break
            elif input == ord('s'):
                print('stop')
                dolly_pub.publish(49)
            print('Reference distance:  ',dolly_sub.get_state())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def move_dolly(dolly_error):
    ## DORY control msg
    ##msg can be interpret as below
    # HERE, use dolly_mode_interpreter func 
    # start-R : 50~99
    # start-L  : 0~48
    # stop         : 49

    test = 1
    while not rospy.is_shutdown():
        try:     
            target_msg = target_marker_sub.get()
            #target point ref Llink0 frame
            # print('get target position in main*****')
            target_ref_link0_point = target_msg.points
            px = target_ref_link0_point[0].x
            py = target_ref_link0_point[0].y 
            pz = target_ref_link0_point[0].z
            if(test == 1 ):
                print('******************PY :     ', py)    
                test = 2
            
            print('Reference distance:  ',dolly_sub.get_state())
            
            ####################################
            ## DORRY MOVE
            ####################################
            ##in case tomato is not recongnized 
            if(px == -1 and py == -1 and pz == -1):
                ##Go to right
                # print('*****go right in no detection')
                dolly_pub.publish(50)
            elif(py > dolly_error):
                # print('L')
                dolly_pub.publish(40)
            elif(-dolly_error > py ):
                # print('R')
                dolly_pub.publish(60)
            else:
                # print('S')
                dolly_pub.publish(49)
                rospy.sleep(0.3)
                break
            rospy.sleep(0.3)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

def move_dolly_by_tracking(max_distance,move_flag):
    thred = 0.05
    thred_over_cnt = 0
    dolly_pub.publish(49)
    while(thred_over_cnt < 5):
        print('thred_over_cnt',thred_over_cnt)
        target_msg = target_marker_sub.get()
        target_ref_link0_point = target_msg.points
        px = target_ref_link0_point[0].x
        py = target_ref_link0_point[0].y 
        pz = target_ref_link0_point[0].z
        current_distance = dolly_sub.get_state()
        if(py != -1):
            target_y = py
        else:
            target_y = 1
        print(target_y)
        D_isOK = bool(math.fabs(target_y) < thred)
        ###########################################
        ## when dolly was about to out of the lane 
        ###########################################
        if (current_distance < -0.01):
            print('got to righr for a while')
            move_flag['L'] = False
            move_flag['R'] =True
            dolly_pub.publish(49)
            for i in range(10):
                dolly_pub.publish(59)
                rospy.sleep(0.5)
            dolly_pub.publish(49)
            break

        if (current_distance > max_distance):
            print('dolly is about to off the rane')
            print('got to left for a while')
            move_flag['L'] = True
            move_flag['R'] = False
            dolly_pub.publish(49)
            for i in range(10):
                dolly_pub.publish(39)
                rospy.sleep(0.5)
            dolly_pub.publish(49)
            break

        if (move_flag['Force'] and move_flag['L'] and not move_flag['R']):
            print('got to left for a while')
            move_flag['L'] = True
            move_flag['R'] =True
            move_flag['Force'] = False
            dolly_pub.publish(49)
            for i in range(5):
                dolly_pub.publish(59)
                rospy.sleep(0.2)
            dolly_pub.publish(49)
            break
        elif (move_flag['Force'] and move_flag['R'] and not move_flag['L']):
            print('got to righr for a while')
            move_flag['L'] = True
            move_flag['R'] =True
            move_flag['Force'] = False
            dolly_pub.publish(49)
            for i in range(5):
                dolly_pub.publish(39)
                rospy.sleep(0.2)
            dolly_pub.publish(49)
            break

        ###########################################
        ## usual state
        ###########################################
        if(D_isOK):
            print('stop')
            thred_over_cnt  += 1
            speed_control = 49
            dolly_pub.publish(49)
            rospy.sleep(0.2)
            if(thred_over_cnt >= 5):
                break
        else:
            d_err = -target_y
            if (d_err > 0.5):
                d_err = 0.5
            elif (d_err < -0.5):
                d_err = -0.5

            ###############################################
            ##check move flag
            ###############################################
            if(not move_flag['L'] and d_err<0):
                d_err = -d_err
            if(not move_flag['R'] and d_err>0):
                d_err = -d_err
            ##########################################

            speed_control = 49 + 20 * d_err 
            
            dolly_pub.publish(UInt16(speed_control))
        rospy.sleep(0.15)
    return move_flag

def move_dolly_right():
    thred = 0.05
    thred_over_cnt = 0
    dolly_pub.publish(49)
    while(thred_over_cnt < 3):
        print('thred_over_cnt',thred_over_cnt)
        target_msg = target_marker_sub.get()
        target_ref_link0_point = target_msg.points
        px = target_ref_link0_point[0].x
        py = target_ref_link0_point[0].y 
        pz = target_ref_link0_point[0].z
        current_distance = dolly_sub.get_state()
        if(py != -1):
            target_y = py
        else:
            target_y = -1
        print(target_y)

        D_isOK = bool(math.fabs(target_y) < thred)
        ###########################################
        ## usual state
        ###########################################
        if(D_isOK):
            print('stop')
            thred_over_cnt  += 1
            speed_control = 49
            dolly_pub.publish(49)
            rospy.sleep(0.2)

        else:
            d_err = -target_y
            if (d_err > 0.5):
                d_err = 0.5
            elif (d_err < -0.5):
                d_err = -0.5

            speed_control = 49 + 20 * d_err 
            
            dolly_pub.publish(UInt16(speed_control))
        rospy.sleep(0.15)

def move_dolly_left():
    thred = 0.05
    thred_over_cnt = 0
    dolly_pub.publish(49)
    while(thred_over_cnt < 3):
        print('thred_over_cnt',thred_over_cnt)
        target_msg = target_marker_sub.get()
        target_ref_link0_point = target_msg.points
        py = target_ref_link0_point[0].y 
        current_distance = dolly_sub.get_state()
        if(py != -1):
            d_err = -py
            if(py < -0.10):
                d_err = -1
        else:
            d_err = -1
            #ipdb.set_trace()()()()()()()


        D_isOK = bool(math.fabs(d_err) < thred)
        ###########################################
        ## usual stated_err
        ###########################################
        if(D_isOK):
            print('stop')
            thred_over_cnt  += 1
            speed_control = 49
            dolly_pub.publish(49)
            rospy.sleep(0.2)

        else:
            if (d_err > 0.5):
                d_err = 0.5
            elif (d_err < -0.5):
                d_err = -0.5

            speed_control = 49 + 20 * d_err 
            
            dolly_pub.publish(UInt16(speed_control))
        rospy.sleep(0.15)

def move_dolly_by_fb():
    thred = 0.03
    thred_over_cnt = 0
    target_y = 1
    ##########IMPORTANT#############
    #### initialize the offset of distance 
    #### to check get reference movement in this loop
    while(thred_over_cnt < 5):
        print('Reference distance:  ',dolly_sub.get_local_state())
        if(target_y == 1):
            ############################
            #get target-msg only 1time
            target_msg = target_marker_sub.get()
            target_ref_link0_point = target_msg.points
            px = target_ref_link0_point[0].x
            py = target_ref_link0_point[0].y 
            pz = target_ref_link0_point[0].z
            if(py != -1):
                target_y = py
        print('target_y:  ',target_y)
        D_isOK = bool(math.fabs(target_y + dolly_sub.get_local_state()) < thred)
        if(D_isOK):
            print('stop')
            thred_over_cnt  += 1
            speed_control = 49
            dolly_pub.publish(49)
        else:
            d_err = -(target_y + dolly_sub.get_local_state() )
            print(d_err)
            if (d_err > 0.6):
                d_err = 0.6
            elif (d_err < -0.6):
                d_err = -0.6
            speed_control = 49 + 22 * d_err 
            dolly_pub.publish(UInt16(speed_control))
        rospy.sleep(0.15)

if __name__ == '__main__': 
    ######################################
    ## init_node & create arm commander
    ######################################
    rospy.init_node('Arm_main',anonymous=True)
    LArm = arm_master.Arm_state_commander('L')
    RArm = arm_master.Arm_state_commander('R')

    ######################################
    # here describe the listener of target
    ######################################
    target_marker_node = "/target_marker_Llink0_frame"
    target_marker_sub = target_sub.target_subscriber('L')

    ######################################
    ## instance to get feedback from Dolly
    ######################################
    dolly_pub = rospy.Publisher("Dolly/command", UInt16, queue_size = 1)
    dolly_sub = DollyFB.Dolly_feedback()

    ####################
    ## define max dist of dolly
    ####################
    dolly_max_distance = 1.2
    move_direction_flag = {'L':True,'R':True,'Force':False}


    while not rospy.is_shutdown():
        try:
            # key_control()
            # continue

            print('start main loop')
            # print('move_direction_flag: ',move_direction_flag)
            # move_dolly(0.05)
            # move_dolly_by_fb()
            # move_direction_flag = move_dolly_by_tracking(dolly_max_distance,move_direction_flag)

            #move dolly
            move_dolly_left()

            print('start_moving')
            
            #wait until the state of arm is changed
            while(LArm.arm_response == 'WAIT'):
                ##command to move arm
                LArm.start_moving()
                rospy.sleep(0.1)

            #wait until the arm process finish
            while (LArm.arm_response == 'OTW'):
                rospy.sleep(0.1)

            #get the result of arm to decide next plan
            arm_result = LArm.arm_response   
            print('arm_result   ** ', arm_result)  
            print('*******************[main.py]stop_moving')

            #stop arm
            while(LArm.arm_response != 'WAIT'):
                LArm.stop_moving()
                rospy.sleep(0.1)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
