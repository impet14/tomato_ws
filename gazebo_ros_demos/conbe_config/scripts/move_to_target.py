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
import handeye_detect as HandEye


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

    eef_joint = DXL.DXL_CONTROL(control_joint='joint6_controller')
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
    #################################
    #define the initial HSV param for mask1&2
    Lmask_MIN = np.array([0,101,144])
    Lmask_MAX = np.array([7,255,240])
    Umask_MIN = np.array([170,160,104])
    Umask_MAX = np.array([180,255,206])
    #make instance to create trackbar
    handEyeFeedback = HandEye.ColorExtract(Lmask_MIN,Lmask_MAX,Umask_MIN,Umask_MAX,320,240)

    go_to_ready(conbe_arm)
    eef_joint.open()

    # rospy.sleep(1)
    # eef_joint.close()

    while not rospy.is_shutdown():
        try:
            print "============ Press `Enter` to go to the target position ..."
            raw_input()

            #receive the target msg which is in target_frame
            #get marker at least 5 times 
            target_msg = rospy.wait_for_message(target_marker_node, Marker)
            target_ref_link0_point = target_msg.points

            px = target_ref_link0_point[0].x
            py = target_ref_link0_point[0].y
            pz = target_ref_link0_point[0].z 

            # print('px,py,pz: ',px,py,pz)

            ## need to add pi to yaw rotation to offset from link0 coordination to eef coordination
            #This is fixed value of this robot
            offset = pi

            ##offset_z is the offset value of Z-axiz. which decides the point to calculate
            #  the rotation of axis to decide the orientation of eef
            #This will really affect to the solution of IK
            offset_z = 0.32 if(pz>0.34) else 0.37
            roll  =  0

            pitch,yaw = calc_orientation(px,py,pz,offset_z)

            ##Remember that the rotation will be done from Rot(z)*Rot(y)+Rot(x)
            arm_orientation = tf.transformations.quaternion_from_euler(roll,pitch,yaw+offset)


            #decide the preposition. this will be the destanse between eef and target along with Z-axis
            L = 0.05
            px,py,pz = set_pre_position(arm_orientation,px,py,pz,L)

            ox = arm_orientation[0]
            oy = arm_orientation[1]
            oz = arm_orientation[2]
            ow = arm_orientation[3]

            eef_markerZ = set_marker_param(eef_markerZ,'z',px,py,pz,arm_orientation)
            eef_markerY = set_marker_param(eef_markerY,'y',px,py,pz,arm_orientation)
            eef_markerX = set_marker_param(eef_markerX,'x',px,py,pz,arm_orientation)

            pub.publish(eef_markerZ)
            pub.publish(eef_markerY)
            pub.publish(eef_markerX)

            #GO TO PRE GRIP POSITION 
                ##CHECKING => GO TO NEXT ONLY WHEN SUCCESS
            conbe_arm.go_to_pose_goal(px,py,pz,ox,oy,oz,ow)
            print(handEyeFeedback.msg)

            rospy.sleep(1)

            if(handEyeFeedback.msg['num'] > 3000):
                eef_joint.close()
                rospy.sleep(2)





            # go_to_box(conbe_arm)

            # rospy.sleep(1)

            #GO BACK TO READY POSE
            go_to_ready(conbe_arm)

            continue

            ##TRACKING PROCESS
                ##CHECKING => GO TO NEXT ONLY WHEN SUCCESS

            ##POOLING PROCESS
                ##CHECKING => GO TO NEXT ONLY WHEN SUCCESS

            ##PLACE TOMATO IN THE BOX
                ##CHECKING => GO TO NEXT ONLY WHEN SUCCESS
            
            #GO BACK TO READY POSE
            go_to_ready(conbe_arm)

            eef_joint.close()
            rospy.sleep(1)

            rospy.sleep(0.05)
            go_to_box(conbe_arm)
            eef_joint.open()
            rospy.sleep(1)

            #GO BACK TO READY POSE
            go_to_ready(conbe_arm)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
