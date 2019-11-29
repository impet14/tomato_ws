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

def create_marker(axis):
    marker = Marker()
    marker.header.frame_id = 'Llink0'
    marker.header.stamp = rospy.Time.now()
    marker.ns = "visulize-eef-orientation"
    marker.id = 0 if(axis=='x') else 1 if(axis=='y')  else 2
    marker.action = Marker.ADD
    marker.color.r = 1.0 if(axis=='x') else 0.0
    marker.color.g = 1.0 if(axis=='y') else 0.0
    marker.color.b = 1.0 if(axis=='z') else 0.0
    marker.color.a = 1.0 
    marker.scale.x = 0.1
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    return marker

def set_marker_param(marker,axis,px,py,pz,quaternion):
    ##offest the orientation
    ## conversion to represent each axis. since arrow marker is along with x-axis as a default,
    ## this is the conversion quaternion 
    orientation =  {'x' :[0.0,0.0,0.0,1],
                    'y' :[0.0,0.0,0.7071068,0.707168],
                    'z' :[0.0,-0.7071068,0.0,0.7071068]}
    ###marker to visualize eef arrow
    marker.pose.position.x = px
    marker.pose.position.y = py
    marker.pose.position.z = pz
    #since the original arrow is along with x-axis,need to offset make it same as link0 frame.
    #after this, put the one
    link0_to_EEF = np.asarray(orientation[axis], dtype=np.float32)
    q_new = tf.transformations.quaternion_multiply(quaternion, link0_to_EEF)
    marker.pose.orientation.x=q_new[0]
    marker.pose.orientation.y=q_new[1]
    marker.pose.orientation.z=q_new[2]
    marker.pose.orientation.w=q_new[3]
    marker.lifetime = rospy.Duration()
    marker.type = 0
    return marker

def set_pre_position(q_orientation,px,py,pz,L):
    ##convert to matrix
    matrix = tf.transformations.quaternion_matrix(q_orientation)
    # print('matrix: ',matrix)
    ##set the point
    point = np.matrix([0, 0, L, 1], dtype='float32')
    point.resize((4, 1))
    ##get the point along with eef_frame
    rotated = matrix*point
    ##get the pre-grip position
    pre_position = np.array([ px + rotated.item(0), py + rotated.item(1), pz + rotated.item(2)])
    return pre_position
    
def calc_orientation(px,py,pz,offset_z):
    yaw= math.atan2(py,px)
    z_offset = math.fabs(pz-offset_z)
    Beta = math.atan2(z_offset,math.sqrt(px*px+py*py))
    pitch = Beta - pi/2 if(pz-offset_z > 0) else - Beta - pi/2
    return pitch,yaw


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
        if input == ord('z'):  # if key 'z' is pressed 
            print('q-pressed: Go to right')
            dolly_pub.publish(60)
        elif input == ord('x'):  # if key 'x' is pressed 
            print('x-pressed: Go to left')
            dolly_pub.publish(40)
        elif input == ord('q'): # break
            cv2.destroyAllWindows()
            flag = False
        else :
            print('::stop')
            dolly_pub.publish(49)
        rospy.sleep(0.2)



if __name__ == '__main__': 
    ## init_node & create arm commander
    rospy.init_node('Arm_main',anonymous=True)
    LArm = arm_master.Arm_state_commander('L')

    ###################################
    # Publisher
    ###################################
    ## EEF COORDINARION Visualization
    pub = rospy.Publisher("eef_arrow", Marker, queue_size = 1)
    eef_markerZ = create_marker('z')
    eef_markerY = create_marker('y')
    eef_markerX = create_marker('x')

    ## DORY control msg
    ##msg can be interpret as below
    # HERE, use dolly_mode_interpreter func 
    # start-R : 50~99
    # start-L  : 0~48
    # stop         : 49
    dolly_pub = rospy.Publisher("dolly", UInt16, queue_size = 1)

    ##################################
    # here describe the listener of target
    ##################################
    target_marker_node = "/target_marker_Llink0_frame"
    print("waiting for  --/target_maker_Llink0_frame-- message")

    ##################################
    ###create handeye detect instance
    ##################################
    handeye_w = 640
    handeye_h = 480
    handEyeFeedback =HandEye.handeye(width=handeye_w,height=handeye_h)
    

    while not rospy.is_shutdown():
        try:
            key_control()
            continue

            print('start main loop')
            ####################################
            ## Wait until the target position
            ## will be stable
            ####################################
            # #receive the target msg which is in target_frame
            # #get marker at least 5 times ?? haven't implemented yet


            rospy.wait_for_message(target_marker_node, Marker)
            px = -1
            offset_z = 0.07
            while px < 0.01 : 
                target_msg = rospy.wait_for_message(target_marker_node, Marker)
                px = target_msg.points[0].x
                rospy.sleep(0.1) 
            
            error = 100        
            cnt_err = 0  
            tmp_pz = 0
            while cnt_err < 5:
                target_msg = rospy.wait_for_message(target_marker_node, Marker)
                
                if (math.fabs(tmp_pz - target_msg.points[0].z) > 0.015):
                    cnt = 0

                tmp_pz = target_msg.points[0].z
                cnt_err+=1
                rospy.sleep(0.1)
            
            #target point ref Llink0 frame
            print('get target position in main*****')
            target_ref_link0_point = target_msg.points
            px = target_ref_link0_point[0].x
            py = target_ref_link0_point[0].y 
            pz = target_ref_link0_point[0].z + offset_z ##0.04 

            ####################################
            ## DORRY MOVE
            ####################################
            print('py : ', py)

            range = 0.01

            if(py > 0.42):
                print('start-L')
                dolly_pub.publish(0)
                continue
            elif(0.42 >= py > range):
                print('start-L')
                dolly_pub.publish(UInt16(-153.125 * py + 64.3125))
                continue
            elif(range >= py >= -range):
                print('stop')
                dolly_pub.publish(49)
                rospy.sleep(1)
            elif(-range > py >= -0.42):
                print('start-R')
                dolly_pub.publish(UInt16(-156.25 * py + 33.375))
                continue
            else:
                print('start-R')
                dolly_pub.publish(UInt16(99))
                continue

            ##################
            ##command to move arm
            ##################
            print('start_moving')
            LArm.start_moving()
            print('stop_moving')
            LArm.stop_moving()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
