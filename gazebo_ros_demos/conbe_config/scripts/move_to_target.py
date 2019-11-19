#!/usr/bin/env python

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
################
import numpy as np
import math
import rospy
import tf
from geometry_msgs.msg import Quaternion
import rospy
from visualization_msgs.msg import Marker
import move_group_python_interface_tutorial as MoveGroupIF
import dxl_move as DXL
######hand eye########
import cv2
from cv_bridge import CvBridge, CvBridgeError
# import handeye_detect as HandEye
import handeye_sub as HandEye
import jog_move as JogMove


def create_marker(axis):
    marker = Marker()
    marker.header.frame_id = 'link0'
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

    rotated = matrix*point
    pre_position = np.array([ px + rotated.item(0), py + rotated.item(1), pz + rotated.item(2)])

    return pre_position
    
def calc_orientation(px,py,pz,offset_z):
    yaw= math.atan2(py,px)
    z_offset = math.fabs(pz-offset_z)

    Beta = math.atan2(z_offset,math.sqrt(px*px+py*py))
    pitch = Beta - pi/2 if(pz-offset_z > 0) else - Beta - pi/2
    return pitch,yaw

def go_to_ready(conbe_arm):
    joint_angle = np.empty(6)
    joint_angle[0] = 0
    joint_angle[1] = 1.1760
    joint_angle[2] = 0
    joint_angle[3] = 1.237
    joint_angle[4] = 0
    joint_angle[5] = 1.4214
    conbe_arm.go_to_joint_state(joint_angle)

def go_to_box(conbe_arm):
    joint_angle = np.empty(6)
    joint_angle[0] = -1.57075
    joint_angle[1] = -0.38
    joint_angle[2] = 0
    joint_angle[3] = 0.4244
    joint_angle[4] = 0
    joint_angle[5] = 1.95
    conbe_arm.go_to_joint_state(joint_angle)

if __name__ == '__main__': 

    ##########################################
    ## create instance to control single motor
    ##########################################
    eef_joint = DXL.DXL_CONTROL(control_joint='joint6_controller')


    #########################################
    ## create instance to control the arm
    #########################################
    conbe_arm = MoveGroupIF.MoveGroupPythonIntefaceTutorial()

    ###################################
    # try to visualize the arrow to debug
    ###################################
    pub = rospy.Publisher("eef_arrow", Marker, queue_size = 1)
    eef_markerZ = create_marker('z')
    eef_markerY = create_marker('y')
    eef_markerX = create_marker('x')

    ##################################
    # here describe the listener
    ##################################
    target_marker_node = "/target_marker_link0_frame"
    print("waiting for  --/target_maker_link0_frame-- message")

    ##################################
    ###create handeye detect instance
    ##################################
    handeye_w = 640
    handeye_h = 480
    handEyeFeedback =HandEye.handeye(width=handeye_w,height=handeye_h)
    
    ################################
    ####create jogging mode instance
    ################################
    frame_id = 'EEFlink'
    group_name = 'conbe'
    link_name = 'EEFlink'
    linear_delta = 0.012
    angular_delta = 0.01
    jog = JogMove.jog_control(frame_id=frame_id,group_name=group_name,link_name=link_name,linear_delta=linear_delta,angular_delta=angular_delta)

    #####################
    ## MOVE :: READY POSE
    #####################
    go_to_ready(conbe_arm)
    eef_joint.open()

    while not rospy.is_shutdown():
        try:
            print "============ Press `Enter` to go to the target position ..."
            raw_input()

            ###################################
            ### MOVE :: First Approach 
            ###################################
            # #receive the target msg which is in target_frame
            # #get marker at least 5 times ?? haven't implemented yet
            
            target_msg = rospy.wait_for_message(target_marker_node, Marker)
            target_ref_link0_point = target_msg.points

            px = target_ref_link0_point[0].x -0.015
            py = target_ref_link0_point[0].y + 0.015
            pz = target_ref_link0_point[0].z + 0.07 ##0.04 

            ## need to add pi to yaw rotation to offset from link0 coordination to eef coordination
            #This is fixed value of this robot
            offset = pi

            ##offset_z is the offset value of Z-axiz. which decides the point to calculate
            #  the rotation of axis to decide the orientation of eef
            ## This will directly affect to the solution of IK
            offset_z = 0.32 if(pz>0.34) else 0.37
            roll  =  0
            pitch,yaw = calc_orientation(px,py,pz,offset_z)

            ##Remember that the rotation will be done from Rot(z)*Rot(y)+Rot(x)
            arm_orientation = tf.transformations.quaternion_from_euler(roll,pitch,yaw+offset)

            #decide the preposition. this will be the destanse between eef and target along with Z-axis
            L = -0.01 #-0.005
            px,py,pz = set_pre_position(arm_orientation,px,py,pz,L)

            ox = arm_orientation[0]
            oy = arm_orientation[1]
            oz = arm_orientation[2]
            ow = arm_orientation[3]

            eef_markerZ = set_marker_param(eef_markerZ,'z',px,py,pz,arm_orientation)
            eef_markerY = set_marker_param(eef_markerY,'y',px,py,pz,arm_orientation)
            eef_markerX = set_marker_param(eef_markerX,'x',px,py,pz,arm_orientation)

            # pub.publish(eef_markerZ)
            # pub.publish(eef_markerY)
            # pub.publish(eef_markerX)

            #GO TO PRE GRIP POSITION 
            current_pose = conbe_arm.go_to_pose_goal(px,py,pz,ox,oy,oz,ow)
            rospy.sleep(3)

            # print('GOAL POSITION: ',px,py,pz)
            # print('EEF  POSITION: ',current_pose.position)

            x_error = px - current_pose.position.x 
            y_error = py - current_pose.position.y 
            z_error = pz - current_pose.position.z 
            # print('x error: ', x_error)
            # print('y error: ', y_error)
            # print('z error: ', z_error)

            #if the error is too big  : in case that the IK couldn't be calculated
            if(math.sqrt(px**2 + py**2 + pz**2) > 0.15):
                print('*********Cannot proceed to the next phase. Calculate again')
                continue

            #################################################
            ## MOVE :: Tracking 
            #################################################

            thred = 200000

            #### approach untill the handeye cam filled more than thredshod
            # print(handEyeFeedback.w,handEyeFeedback.h,handEyeFeedback.num)

            w_location = handEyeFeedback.w - handeye_w/2
            h_location = handEyeFeedback.h - handeye_h/2

            #In case that the handeye-cam cannot see the target at all
            ##############################
            ## MOVE :: LOOK FOR THE TARGET
            ##############################
       

            #Handye cam detect the target 
            ################################
            ## MOVE :: APPROACH TO THE TARGET
            ################################

            def adjust_h_w(h_loc,w_loc):
                delta_h =  1.0 if (h_loc < 0 ) else 0 if (h_loc == -1) else -1.0
                delta_w = -1.0 if (w_loc < 0 ) else 0 if (w_loc == -1) else  1.0
                jog.set_linear_axis(x=delta_h,y=delta_w,z=0)
                jog.move_with_linear_delta()
                # jog.set_angular_axis(x=-delta_w,y=delta_h,z=0)
                # jog.move_with_angular_delta()
                rospy.sleep(0.5)
                return (handEyeFeedback.h - handeye_h/2,handEyeFeedback.w - handeye_w/2)
        
            def adjust_z():
                delta = 1.0
                jog.set_linear_axis(x=0,y=0,z=delta)
                jog.move_with_linear_delta()
                rospy.sleep(0.5)
                return (handEyeFeedback.h - handeye_h/2,handEyeFeedback.w - handeye_w/2)
                        
            range = 50
            count = 1

            try:
                while(handEyeFeedback.num < thred):
                    count += count * 0.1
                    range += count
                    H_state = bool(math.fabs(h_location) > handeye_h/range)
                    W_state = bool(math.fabs(w_location) > handeye_w/range) 
                    if(H_state and W_state):
                        print('W-H-adjust: ',h_location,w_location)
                        h_location, w_location = adjust_h_w(h_location,w_location)
                    elif (H_state):
                        ###Adjust height direction
                        print('H_adjust: ',h_location)
                        h_location, w_location = adjust_h_w(h_location,-1)
                    elif (W_state):
                        ###Adjust height direction
                        print('W_adjust: ',w_location)
                        h_location, w_location = adjust_h_w(-1,w_location)
                    else:
                        print('Z-adjust')
                        ###approach to the target
                        h_location, w_location = adjust_z()
                    if(count > 10):
                        print('Z-adjust')
                        ###approach to the target
                        h_location, w_location = adjust_z()  
                    if(count > 50):
                        break
            except KeyboardInterrupt:
                print 'interrupted!'
                break

 
            rospy.sleep(2.0)
            eef_joint.close()
            rospy.sleep(2.0)

            ################################
            ##MOVE :: GO TOBOX , GO TO READY
            ################################

            ###if grab correctly 
            if(handEyeFeedback.num > thred):
                go_to_box(conbe_arm)
                eef_joint.open()
                rospy.sleep(1)
                go_to_ready(conbe_arm)
            else:
                #GO BACK TO READY POSE
                go_to_ready(conbe_arm)
                eef_joint.open()
            
            print('continue')
            continue


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
