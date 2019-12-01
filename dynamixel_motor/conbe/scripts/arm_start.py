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
from geometry_msgs.msg import Quaternion
from cv_bridge         import CvBridge, CvBridgeError
from visualization_msgs.msg       import Marker
from utils import dxl_move as DXL
from utils import handeye_sub as HandEye
from utils import conbe_ik as CONBE
from utils import client_trajectory
from utils import arm_state_manager as arm_slave
from utils import eef_tf_listener


def create_marker(axis,LorR):
    marker = Marker()
    marker.header.frame_id = LorR + 'link0'
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

def init_pose(conbe_arm):
    joint_angle = np.empty(6)
    joint_angle[0] = 0
    joint_angle[1] = 0
    joint_angle[2] = 0
    joint_angle[3] = 0
    joint_angle[4] = 0
    joint_angle[5] = 0
    conbe_arm.move(joint_angle)

def go_to_ready(conbe_arm):
    joint_angle = np.empty(6)
    joint_angle[0] = 0
    joint_angle[1] = 1.1760
    joint_angle[2] = 0
    joint_angle[3] = 1.237
    joint_angle[4] = 0
    joint_angle[5] = 1.4214
    conbe_arm.move(joint_angle)

def go_to_box(conbe_arm):
    joint_angle = np.empty(6)
    joint_angle[0] = -1.57075
    joint_angle[1] = -0.38
    joint_angle[2] = 0
    joint_angle[3] = 0.4244
    joint_angle[4] = 0
    joint_angle[5] = 1.95
    conbe_arm.move(joint_angle)

def dolly_mode_interpreter(command):
    if(command == 49):
        print('stop')
    elif(command < 49):
        print('left')
    else:
        print('right')

def eef_open(eef):
    eef.move_to_goal(0.20)

def eef_close(eef):
    eef.move_to_goal(-1.3) #right arm

if __name__ == '__main__': 

    rospy.init_node('Arm_interface', anonymous=True)
    LorR = 'L'

    Arm = arm_slave.Arm_state_manager(LorR)
    ##########################################
    ## create instance to calculate IK
    ##########################################
    pos_bound = [0.01,0.01,0.01]
    ori_bound = [0.1,0.1,0.1]

    conbe_ik   = CONBE.ConbeIK(urdf_param='/' + LorR +'Arm/robot_description',LorR=LorR,pos_bound=pos_bound,ori_bound=ori_bound)
    conbe_ik.check_setting()

    ########################################
    ## create instance to calculate tf from link0 to EEF
    ########################################
    eef_jog = eef_tf_listener.eef_jog(LorR)

    ##########################################
    ## create instance to control single motor
    ##########################################
    eef_joint = DXL.DXL_CONTROL(control_joint='/' + LorR +'Arm/joint6_controller')
    roll_joint0  = DXL.DXL_CONTROL(control_joint='/' + LorR +'Arm/joint0_controller') 
    pitch_joint3 = DXL.DXL_CONTROL(control_joint='/' + LorR +'Arm/joint3_controller')
    pitch_joint5 = DXL.DXL_CONTROL(control_joint='/' + LorR +'Arm/joint5_controller')

    #########################################
    ## create instance to control the arm using FollowJointTrajectory
    #########################################
    conbe_arm = client_trajectory.Joint('/' +  LorR +'Arm/conbe' + LorR + '_controller') 

    ###################################
    # Publisher
    ###################################
    ## EEF COORDINARION Visualization
    pub = rospy.Publisher("eef_arrow", Marker, queue_size = 1)
    eef_markerZ = create_marker('z',LorR)
    eef_markerY = create_marker('y',LorR)
    eef_markerX = create_marker('x',LorR)

    ##################################
    # here describe the listener
    ##################################
    target_marker_node = "/target_marker_" + LorR + "link0_frame"

    ##################################
    ###create handeye detect instance
    ##################################
    handeye_w = 640
    handeye_h = 480
    handEyeFeedback =HandEye.handeye(LorR=LorR,width=handeye_w,height=handeye_h)
    
    ## start
    ## MOVE :: READY POSE
    go_to_ready(conbe_arm)
    eef_open(eef_joint)

    Arm.publish_state('WAIT')

    while not rospy.is_shutdown():
        try:
            Arm.publish_state('WAIT')

            #if it's not state to go to target
            if not Arm.get_command():
                continue
            
            Arm.publish_state('OTW')
            ####################################
            ## Wait until the target position
            ## will be stable
            ####################################
            # #receive the target msg which is in target_frame
            # #get marker at least 5 times ?? haven't implemented yet

            rospy.wait_for_message(target_marker_node, Marker)
            px = -1
            offset_z = 0.04
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
            
            #target point ref link0 frame
            print('get target***' + LorR + 'Arm***')
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

            ###################################
            ### MOVE :: First Approach 
            ###################################

            ipdb.set_trace()

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
            offset_z = -0.05 #-0.005
            # px,py,pz = set_pre_position(arm_orientation,px,py,pz,L)

            # ox = arm_orientation[0]
            # oy = arm_orientation[1]
            # oz = arm_orientation[2]
            # ow = arm_orientation[3]

            pos = set_pre_position(arm_orientation,px,py,pz,offset_z)
            ori = arm_orientation

            eef_markerZ = set_marker_param(eef_markerZ,'z',px,py,pz,arm_orientation)
            eef_markerY = set_marker_param(eef_markerY,'y',px,py,pz,arm_orientation)
            eef_markerX = set_marker_param(eef_markerX,'x',px,py,pz,arm_orientation)

            pub.publish(eef_markerZ)
            # pub.publish(eef_markerY)
            # pub.publish(eef_markerX)

            #GO TO PRE GRIP POSITION 
            print ('calculate IK*****')
            angles = conbe_ik.calculate(pos,ori)
            if(angles == None):
                print('calculation wasnot succeed')
                Arm.publish_state('C_FAILED')
                continue

            conbe_arm.move(angles)
            rospy.sleep(1)

            ipdb.set_trace()

            ######################################
            #get current pose and compensate error
            ######################################

            #################################################
            ## MOVE :: Tracking 
            #################################################

            thred = 150000
            # thred = 180000


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
            def adjust_z(delta):
                jog.set_linear_axis(x=0,y=0,z=delta)
                jog.move_with_linear_delta()
                rospy.sleep(0.5)

            print('handeyeFD.num = ',handEyeFeedback.num)

            ##look for the target
            if(handEyeFeedback.num < 10):
                handeye_recognition_isOK = False
                for i in range(20):
                    print('look for ...',i)
                    delta = 0.05 if(4 < i and i < 15) else -0.05
                    print(delta)
                    pitch_joint5.move_with_delta(delta)

                    print('handeyeFD.num',handEyeFeedback.num)
                    ipdb.set_trace()
                    #in case that the handeye recognaizes the target
                    if(handEyeFeedback.num != 0):
                        print('handEyeFeedback is ok')
                        handeye_recognition_isOK = True
                        break
            else:
                handeye_recognition_isOK = True

            #if the handeye couldn't find the target
            if(not handeye_recognition_isOK):
                print('handEyeFeedback is not ok')
                ipdb.set_trace()
                continue

            def check_if_H_isOK():
                h_error = math.fabs(handEyeFeedback.h - handeye_h/2)
                return bool(h_error < handeye_h/50)

            def check_if_W_isOK():
                w_error = math.fabs(handEyeFeedback.w - handeye_w/2)
                return bool(w_error < handeye_w/80)

            tracking_try_count = 0
            delta_h = -0.05
            delta_w = -0.02
            thed_over_cnt = 0
            while(thed_over_cnt<5):
                if handEyeFeedback.num > thred:
                    thed_over_cnt = + 1

                tracking_try_count += 1
                H_isOK = check_if_H_isOK()
                W_isOK = check_if_W_isOK()

                if(handEyeFeedback.num > 80000):
                    ipdb.set_trace()
                    print('Z-adjust') 
                    delta = 0.03
                    pos = eef_jog.transform([0,0,delta])
                    ori = arm_orientation

                    for i in range(5):
                        print ('calculate IK*****')
                        angles = conbe_ik.calculate(pos,ori)
                        if(angles == None):
                            print('calculation wasnot succeed')
                            Arm.publish_state('C_FAILED')
                        else:
                            conbe_arm.move(angles)
                            break

                if(not H_isOK):
                    h_err = handeye_h/2 - handEyeFeedback.h 
                    print(h_err)
                    current_joint5_state = pitch_joint5.get_current_pos() 
                    if (h_err > 100):
                        h_err = 100
                    elif (h_err < -100):
                        h_err = -100
                    
                    out_h_control = current_joint5_state+(-h_err*0.001)
                    pitch_joint5.move_to_goal(out_h_control)

                if(not W_isOK):
                    w_err = handeye_w/2 - handEyeFeedback.w 
                    print(w_err)
                    current_joint0_state = roll_joint0.get_current_pos() 
                    if (w_err > 100):
                        w_err = 100
                    elif (w_err < -100):
                        w_err = -100
                    
                    out_w_control = current_joint0_state+(w_err*0.001)
                    roll_joint0.move_to_goal(out_w_control)

                # elif(not H_isOK):
                #     print(delta_h)
                #     prev_h_error    = math.fabs(handEyeFeedback.h - handeye_h/2)
                #     pitch_joint5.move_with_delta(delta_h)
                #     current_h_error = math.fabs(handEyeFeedback.h - handeye_h/2)

                #     if(prev_h_error > current_h_error or math.fabs(prev_h_error-current_h_error) < 10):
                #         delta_h = delta_h
                #     else:
                #         delta_h = -1.0 * delta_h
                    
                #     delta_h = -0.09 if(delta_h <0 ) else 0.02
                
                # elif(not W_isOK):

                #     print(delta_w)
                #     prev_w_error    = math.fabs(handEyeFeedback.w - handeye_w/2)
                #     roll_joint0.move_with_delta(delta_w)
                #     current_w_error = math.fabs(handEyeFeedback.w - handeye_w/2)

                #     if(prev_w_error > current_w_error or math.fabs(prev_w_error-current_w_error) < 5):
                #         delta_w = delta_w
                #     else:
                #         delta_w = -1.0 * delta_w

                rospy.sleep(0.1)
                if(tracking_try_count > 1000):
                    print('****************tracking count over 1000')
                    break
            
            ipdb.set_trace()
            print('*****grab-target')
            rospy.sleep(2.0)
            eef_close(eef_joint)
            rospy.sleep(2.0)

            print('grab correctly')
            go_to_box(conbe_arm)
            eef_open(eef_joint)
            rospy.sleep(2.0)
            go_to_ready(conbe_arm)
            rospy.sleep(2.0)
            go_to_ready(conbe_arm)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
