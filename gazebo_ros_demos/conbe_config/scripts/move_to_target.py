#!/usr/bin/env python
import rospy
import numpy as np
from math import pi
import math
import tf
import cv2
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg      import String
from std_msgs.msg      import UInt16
from geometry_msgs.msg import Quaternion
from cv_bridge         import CvBridge, CvBridgeError
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg       import Marker
##import custom class
import move_group_python_interface_tutorial as MoveGroupIF
import dxl_move as DXL
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

def dolly_mode_interpreter(command):
    return 100 if (command == 'start-R') else 200 if(command == 'start-L') else 0

# def arm_look_for(joint,delta):
#     joint.move_with_delta(delta)

def eef_open(eef):
    eef.move_to_goal(0.34)

def eef_close(eef):
    # eef.move_to_goal(-1.46)
    eef.move_to_goal(-0.60) #right arm

if __name__ == '__main__': 

    ##########################################
    ## create instance to control single motor
    ##########################################
    eef_joint = DXL.DXL_CONTROL(control_joint='joint6_controller')
    roll_joint0  = DXL.DXL_CONTROL(control_joint='joint0_controller') 
    pitch_joint3 = DXL.DXL_CONTROL(control_joint='joint3_controller')
    pitch_joint5 = DXL.DXL_CONTROL(control_joint='joint5_controller')

    #########################################
    ## create instance to control the arm
    #########################################
    conbe_arm = MoveGroupIF.MoveGroupPythonIntefaceTutorial()

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
    # start-R : 100
    # start-L  : 200
    # stop         : 0
    dolly_pub = rospy.Publisher("dolly", UInt16, queue_size = 1)

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
    eef_open(eef_joint)

    while not rospy.is_shutdown():
        try:
            go_to_ready(conbe_arm)
            rospy.sleep(0.5)
            go_to_ready(conbe_arm)

            print "============ Start to go to the target position ..."
            # raw_input()

            # print('=========arduino test=========')
            # print('start-R')
            # dolly_pub.publish(dolly_mode_interpreter('start-R'))
            # rospy.sleep(10)
            # print('stop')
            # dolly_pub.publish(dolly_mode_interpreter('stop'))
            # rospy.sleep(10)
            # print('start-L')
            # dolly_pub.publish(dolly_mode_interpreter('start-L'))
            # rospy.sleep(10)

            # print('stop')
            # dolly_pub.publish(dolly_mode_interpreter('stop'))
            # rospy.sleep(10)
            # break


            # for i in range(20):
            #     print('look for ...',i)
            #     delta = 0.1 if(4 < i and i < 15) else -0.1
            #     # pitch_joint3.move_with_delta(0.02)
            #     print(delta)
            #     pitch_joint5.move_with_delta(delta)
            # break

            ###################################
            ### MOVE :: First Approach 
            ###################################
            # #receive the target msg which is in target_frame
            # #get marker at least 5 times ?? haven't implemented yet
            rospy.wait_for_message(target_marker_node, Marker)
            px = -1
            offset_z = 0.07
            while px < 0.01 : 
                target_msg = rospy.wait_for_message(target_marker_node, Marker)
                px = target_msg.points[0].x
                rospy.sleep(0.5) 
            
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
            
            target_ref_link0_point = target_msg.points
            px = target_ref_link0_point[0].x
            py = target_ref_link0_point[0].y 
            pz = target_ref_link0_point[0].z + offset_z ##0.04 
            rospy.sleep(0.5) 
                



            ##############################################
            ##
            ## HERE need to add the conditional branch
            ## to check if the target is inside the worksp
            ##
            ###############################################


            ## need to add pi to yaw rotation to offset from link0 coordination to eef coordination
            #This is fixed value of this robot
            offset = pi

            ##offset_z is the offset value of Z-axiz. which decides the point to calculate
            #  the rotation of axis to decide the orientation of eef
            ## This will directly affect to the solution of IK
            # offset_z = 0.32 if(pz>0.34) else 0.37
            offset_z = 0.32 if(pz>0.34) else 0.47
            roll  =  0
            pitch,yaw = calc_orientation(px,py,pz,offset_z)

            ##Remember that the rotation will be done from Rot(z)*Rot(y)+Rot(x)
            arm_orientation = tf.transformations.quaternion_from_euler(roll,pitch,yaw+offset)

            #decide the preposition. this will be the destanse between eef and target along with Z-axis
            L = -0.05 #-0.005
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
            print ('going to tomato')
            conbe_arm.go_to_pose_goal(px,py,pz,ox,oy,oz,ow)
            rospy.sleep(1)
            current_pose = conbe_arm.get_current_pose()
            print ('target current pose')
            print (current_pose)
            # print('GOAL POSITION: ',px,py,pz)
            # print('EEF  POSITION: ',current_pose.position)

            x_error = px - current_pose.position.x 
            y_error = py - current_pose.position.y 
            z_error = pz - current_pose.position.z 
            print('x error: ', x_error)
            print('y error: ', y_error)
            print('z error: ', z_error)

            # compensating hand error before gripping
            conbe_arm.go_to_pose_goal(px,py,pz+z_error-offset_z,ox,oy,oz,ow)
            rospy.sleep(1)
            
            # conbe_arm.go_to_pose_goal(px,py,pz+z_error-offset_z,ox,oy,oz,ow)

            current_pose = conbe_arm.get_current_pose()
            print ('compenstated current pose')
            print (current_pose)
            # print('GOAL POSITION: ',px,py,pz)
            # print('EEF  POSITION: ',current_pose.position)

            x_error = px - current_pose.position.x 
            y_error = py - current_pose.position.y 
            z_error = pz - current_pose.position.z 
            print('x error: ', x_error)
            print('y error: ', y_error)
            print('z error: ', z_error)
            #if the error is too big  : in case that the IK couldn't be calculated

            if(math.sqrt(x_error**2 + y_error**2 + z_error**2) > 0.15 and handEyeFeedback.num < 1000):
                print('*****calculation did not succeed ')
                print(math.sqrt(x_error**2 + y_error**2 + z_error**2))
                continue


            print('*****grab-target')
            rospy.sleep(2.0)
            eef_close(eef_joint)
            rospy.sleep(2.0)

            print('grab correctly')
            go_to_box(conbe_arm)

            break
            # print "============ Press `Enter` to go to go back to ready position ..."
            # raw_input()

            # go_to_ready(conbe_arm)
            # continue

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
        
            def adjust_z(delta):
                jog.set_linear_axis(x=0,y=0,z=delta)
                jog.move_with_linear_delta()
                rospy.sleep(0.5)


            print('handeyeFD.num = ',handEyeFeedback.num)

            ##look for the target
            if(handEyeFeedback.num == 0):
                handeye_recognition_isOK = False
                for i in range(20):
                    print('look for ...',i)
                    delta = 0.05 if(4 < i and i < 15) else -0.05
                    print(delta)
                    pitch_joint5.move_with_delta(delta)

                    #in case that the handeye recognaizes the target
                    if(handEyeFeedback.num != 0):
                        print('handEyeFeedback is ok')
                        handeye_recognition_isOK = True
                        break
            else:
                handeye_recognition_isOK = True


            #if the handeye couldn't find the target
            if(not handeye_recognition_isOK):
                continue



            def check_if_H_isOK():
                h_error = math.fabs(handEyeFeedback.h - handeye_h/2)
                return bool(h_error < handeye_h/50)

            count = 0
            delta = -0.05

            while(handEyeFeedback.num < thred):
                count += 1
                H_isOK = check_if_H_isOK()

                if(H_isOK):
                    print('Z-adjust') 
                    if(handEyeFeedback.num < (thred - 100000) ):
                        delta = 3.0
                    else:
                        ###approach to the target
                        delta = 2.5
                    adjust_z(delta)

                else:

                    # print(delta)
                    # prev_num    = handEyeFeedback.num
                    # pitch_joint5.move_with_delta(delta)
                    # current_num = handEyeFeedback.num

                    # print('current_num: ',current_num)
                    # print('prev_num: ',prev_num)

                    # #when lose the target
                    # recover_count = 0
                    # if(prev_num == 0 and current_num ==0):
                    #     for i in range(20):
                    #         print('look for ...',i)
                    #         delta = 0.05 if(4 < i and i < 15) else -0.05
                    #         print(delta)
                    #         pitch_joint5.move_with_delta(delta)

                    #         #in case that the handeye recognaizes the target
                    #         if(handEyeFeedback.num != 0):
                    #             break
                    #         recover_count += 1

                    # if(recover_count == 20):
                    #     print('cannot recognize the target-- go back to home')
                    #     break

                    # if(prev_num < current_num):
                    #     print('go to same direction')
                    #     delta = delta
                    # else:
                    #     print('go to opposite direction')
                    #     delta = -1.0 * delta

                    print(delta)
                    prev_h_error    = math.fabs(handEyeFeedback.h - handeye_h/2)
                    pitch_joint5.move_with_delta(delta)
                    current_h_error = math.fabs(handEyeFeedback.h - handeye_h/2)

                    if(prev_h_error > current_h_error or math.fabs(prev_h_error-current_h_error) < 10):
                        delta = delta
                    else:
                        delta = -1.0 * delta
                    
                    delta = -0.09 if(delta <0 ) else 0.02

                if(count > 50):
                    print('****************tracking count over 50')
                    break



                            
            print('*****grab-target')
            rospy.sleep(2.0)
            eef_close(eef_joint)
            rospy.sleep(2.0)

            print('grab correctly')
            go_to_box(conbe_arm)
            # rospy.sleep(1.0)
            # go_to_box(conbe_arm)
            eef_open(eef_joint)
            rospy.sleep(2.0)
            go_to_ready(conbe_arm)
            rospy.sleep(2.0)
            go_to_ready(conbe_arm)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
