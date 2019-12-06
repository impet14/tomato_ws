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
from utils import eef_tf_listener
import target_sub

class Manipulator():
    def __init__(self,LorR,handeye_w,handeye_h):
        self.LorR = LorR
        self.arm_interpret_dict = {0:'WAIT',1:'OTW',-1:'NO_TOMATO',-2:'C_FAILED',2:'SUCCEED',5:'T_FAILED'}
        self.state = 'WAIT'
        self.handeye_w = handeye_w
        self.handeye_h = handeye_h
        ##########################################
        ## create instance to calculate IK
        ##########################################
        pos_bound = [0.01,0.01,0.01]
        ori_bound = [0.5,0.5,0.5]

        self.conbe_ik   = CONBE.ConbeIK(urdf_param='/' + LorR +'Arm/robot_description',LorR=LorR,pos_bound=pos_bound,ori_bound=ori_bound)
        self.conbe_ik.check_setting()

        ########################################
        ## create instance to calculate tf from link0 to EEF
        ########################################
        self.eef_jog = eef_tf_listener.eef_jog(LorR)

        ##########################################
        ## create instance to control single motor
        ##########################################
        self.eef_joint = DXL.DXL_CONTROL(control_joint='/' + LorR +'Arm/joint6_controller')
        self.roll_joint0  = DXL.DXL_CONTROL(control_joint='/' + LorR +'Arm/joint0_controller') 
        self.pitch_joint3 = DXL.DXL_CONTROL(control_joint='/' + LorR +'Arm/joint3_controller')
        self.pitch_joint5 = DXL.DXL_CONTROL(control_joint='/' + LorR +'Arm/joint5_controller')

        #########################################
        ## create instance to control the arm using FollowJointTrajectory
        #########################################
        self.conbe_arm = client_trajectory.Joint('/' +  self.LorR +'Arm/conbe' + self.LorR + '_controller') 

        ###################################
        # Publisher
        ###################################
        ## EEF COORDINARION Visualization
        self.pub = rospy.Publisher("eef_arrow", Marker, queue_size = 1)
        self.eef_markerZ = self.create_marker('z',LorR)
        self.eef_markerY = self.create_marker('y',LorR)
        self.eef_markerX = self.create_marker('x',LorR)

        ##################################
        # here describe the listener
        ##################################
        # self.target_marker_node = "/target_marker_" + LorR + "link0_frame"

        self.target_marker_sub = target_sub.target_subscriber(LorR)

        ##################################
        ###create handeye detect instance
        ##################################
        self.handEyeFeedback =HandEye.handeye(LorR=LorR,width=handeye_w,height=handeye_h)

    def set_speed_parameters(self):
        joint_speed = [1.0,0.5,1.0,1.0,1.0,1.0,0.5]
        for i in range(7):
            rospy.set_param('/' + self.LorR + 'Arm/joint' +str(i)+ '_controller/joint_speed',joint_speed[i])

    def create_marker(self,axis,LorR):
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

    def set_marker_param(self,marker,axis,px,py,pz,quaternion):
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

    def adjust_z(self,delta):
        self.eef_jog.set_linear_axis(x=0,y=0,z=delta)
        self.eef_jog.move_with_linear_delta()

    def set_pre_position(self,q_orientation,px,py,pz,L):
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
        
    def calc_orientation(self,px,py,pz,offset_z):
        yaw= math.atan2(py,px)
        z_offset = math.fabs(pz-offset_z)
        Beta = math.atan2(z_offset,math.sqrt(px*px+py*py))
        pitch = Beta - pi/2 if(pz-offset_z > 0) else - Beta - pi/2
        return pitch,yaw

    def init_pose(self):
        joint_angle = np.empty(6)
        joint_angle[0] = 0
        joint_angle[1] = 0
        joint_angle[2] = 0
        joint_angle[3] = 0
        joint_angle[4] = 0
        joint_angle[5] = 0
        self.conbe_arm.move(joint_angle)

    def go_to_ready(self):
        joint_angle = np.empty(6)
        joint_angle[0] = 0
        joint_angle[1] = 1.1760
        joint_angle[2] = 0
        joint_angle[3] = 1.237
        joint_angle[4] = 0
        joint_angle[5] = 1.4214
        self.conbe_arm.move(joint_angle)

    def go_to_box(self):
        joint_angle = np.empty(6)
        joint_angle[0] = -1.57075
        joint_angle[1] = -0.38
        joint_angle[2] = 0
        joint_angle[3] = 0.4244
        joint_angle[4] = 0
        joint_angle[5] = 1.95
        self.conbe_arm.move(joint_angle)

    def check_ws(self,px,py,pz):
        state = True
        # if(px < 0.05 or 0.45 > px):
        #     state = False
        #     print('x-direction: out of worksp')
        # if(math.fabs(py) > 0.30):
        #     state = False
        #     print('y-direction: out of worksp')
        distance = math.sqrt(px**2 + py**2 + (pz-0.325) **2)
        if(distance > 0.50 or distance < 0.05):
            state = False
            print('x:{} ,y:{} ,z:{}'.format(px,py,pz))
            print('target-distance: out of worksp distance : ', distance)
        return state

    def eef_open(self):
        self.eef_joint.move_to_goal(0.20)

    def eef_close(self):
        self.eef_joint.move_to_goal(-1.7) 

    def key_control(self,arm_orientation):
        background = np.zeros((200,300,3), np.uint8)
        cv2.namedWindow('input_test')
        delta = 0.03
        while not rospy.is_shutdown():
            try: 
                cv2.imshow('jog_z_test',background)
                input = cv2.waitKey(0)
                if input == ord('f'):
                    print('f-pressed: Go Forward')

                    pos = self.eef_jog.transform([0,0,delta])
                    ori = arm_orientation

                    for i in range(5):
                        print ('calculate IK*****')
                        angles = self.conbe_ik.calculate(pos,ori)
                        if(angles == None):
                            print('calculation wasnot succeed')
                        else:
                            conbe_arm.move(angles)
                            break

                elif input == ord('b'):  # if key 'x' is pressed 
                    print('b-pressed: Go Backward')

                    pos = self.eef_jog.transform([0,0,-delta])
                    ori = arm_orientation

                    for i in range(5):
                        print ('calculate IK*****')
                        angles = self.conbe_ik.calculate(pos,ori)
                        if(angles == None):
                            print('calculation wasnot succeed')
                        else:
                            self.conbe_arm.move(angles)
                            break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def check_if_H_isOK(self):
        h_error = math.fabs(self.handEyeFeedback.h - self.handeye_h/2)
        return bool(h_error < self.handeye_h/50)

    def check_if_W_isOK(self):
        w_error = math.fabs(self.handEyeFeedback.w - self.handeye_w/2)
        return bool(w_error < self.handeye_w/80)

    def main(self):
        ###set speed params
        self.set_speed_parameters()
        ## start
        self.state = 'OTW'
        ## MOVE :: READY POSE
        self.go_to_ready()
        # init_pose(conbe_arm)
        self.eef_open()
        rospy.sleep(0.5)

        ####################################
        ## Wait until the target position
        ## will be stable
        ####################################
        # #receive the target msg which is in target_frame
        # #get marker at least 5 times ?? haven't implemented yet

        px = -1
        offZ = 0.05
        
        error = 100        
        cnt_err = 0  
        tmp_pz = 0


        ###########
        while cnt_err < 5:
            # target_msg = rospy.wait_for_message(self.target_marker_node, Marker)
            px,py,pz = self.target_marker_sub.getXYZ()
            if (math.fabs(tmp_pz - pz) > 0.015):
                cnt = 0

            tmp_pz = pz
            cnt_err+=1
            rospy.sleep(0.1)

        pz = pz + offZ
        ###########

        # px = 0.30
        # py = 0
        # pz = 0.39
        
        #target point ref link0 frame
        print('get target***' + self.LorR + 'Arm***')

        ##############################################
        ## HERE need to add the conditional branch
        ## to check if the target is inside the worksp
        ###############################################
        if(self.check_ws(px,py,pz)):
            self.state = 'OTW'
        else:   
            self.state = 'NO_TOMATO'
            return self.state
            
        ###################################
        ### MOVE :: First Approach 
        ###################################

        ##ipdb.set_trace()()()

        ## need to add pi to yaw rotation to offset from link0 coordination to eef coordination
        #This is fixed value of this robot
        # offset = pi

        ##offset_z is the offset value of Z-axiz. which decides the point to calculate
        #  the rotation of axis to decide the orientation of eef
        ## This will directly affect to the solution of IK
        # offset_z = 0.32 if(pz>0.34) else 0.37
        offset_z = 0.42 if(pz<0.42) else 0.33
        roll  =  0
        pitch,yaw = self.calc_orientation(px,py,pz,offset_z)

        ##Remember that the rotation will be done from Rot(z)*Rot(y)+Rot(x)
        arm_orientation = tf.transformations.quaternion_from_euler(roll,pitch,yaw+pi)

        ####################
        ## set pre-position
        #decide the preposition. this will be the destanse between eef and target along with Z-axis
        #####################
        L = -0.03 #-0.02        
        pos = self.set_pre_position(arm_orientation,px,py,pz,L)

        ori = arm_orientation

        self.eef_markerZ = self.set_marker_param(self.eef_markerZ,'z',px,py,pz,arm_orientation)
        self.eef_markerY = self.set_marker_param(self.eef_markerY,'y',px,py,pz,arm_orientation)
        self.eef_markerX = self.set_marker_param(self.eef_markerX,'x',px,py,pz,arm_orientation)

        self.pub.publish(self.eef_markerZ)
        # self.pub.publish(self.eef_markerY)
        # self.pub.publish(self.eef_markerX)

        #GO TO PRE GRIP POSITION 
        # print ('calculate IK*****')
        ik_take_time = rospy.get_time()
        angles = self.conbe_ik.calculate(pos,ori)
        ik_take_time = rospy.get_time() - ik_take_time
        print ('ik_take_time',ik_take_time)
        
        if(angles == None):
            print('calculation wasnot succeed')
            self.state = 'C_FAILED'
            self.go_to_ready()
            return self.state

        self.conbe_arm.move(angles)
        rospy.sleep(1)

        ###########go to real target#########
        pos = [px,py,pz]

        ik_take_time = rospy.get_time()
        angles = self.conbe_ik.calculate(pos,ori)
        ik_take_time = rospy.get_time() - ik_take_time
        print ('ik_take_time',ik_take_time)
        
        if(angles == None):
            print('calculation wasnot succeed')
            self.state = 'C_FAILED'
            self.go_to_ready()
            return self.state

        self.conbe_arm.move(angles)
        rospy.sleep(1)

        #######################################


        # key_control()

        ######################################
        #get current pose and compensate error
        ######################################

        #################################################
        ## MOVE :: Tracking 
        #################################################
        # thred = 150000
        # thred = 180000
        # thred = 280 
        thred = 120


        #### approach untill the handeye cam filled more than thredshod
        # print(self.handEyeFeedback.w,self.handEyeFeedback.h,self.handEyeFeedback.r)

        w_location = self.handEyeFeedback.w - self.handeye_w/2
        h_location = self.handEyeFeedback.h - self.handeye_h/2

        #In case that the handeye-cam cannot see the target at all
        ##############################
        ## MOVE :: LOOK FOR THE TARGET
        ##############################

        
        #Handye cam detect the target 
        ################################
        ## MOVE :: APPROACH TO THE TARGET
        ################################        

        print('self.handEyeFeedback.r = ',self.handEyeFeedback.r)

        ##look for the target
        if(self.handEyeFeedback.r < 5):
            handeye_recognition_isOK = False
            for i in range(20):
                print('look for ...',i)
                delta = 0.04 if(4 < i and i < 15) else -0.04
                print(delta)
                self.pitch_joint5.move_with_delta(delta)

                print('handEyeFeedback.num',self.handEyeFeedback.r)
                ##ipdb.set_trace()()()
                #in case that the handeye recognaizes the target
                if(self.handEyeFeedback.r != 0):
                    print('self.handEyeFeedback is ok')
                    handeye_recognition_isOK = True
                    break
        else:
            handeye_recognition_isOK = True

        #if the handeye couldn't find the target
        if(not handeye_recognition_isOK):
            print('self.handEyeFeedback is not ok')
            self.go_to_ready()
            self.state = ('T_FAILED')
            return self.state
        

        tracking_try_count = 0
        delta_h = -0.05
        delta_w = -0.02
        thed_over_cnt = 0
        cnt_delta = 0

        current_joint5_state = self.pitch_joint5.get_current_pos()
        while(thed_over_cnt < 10 and  self.handEyeFeedback.cnt < 30 and not rospy.is_shutdown()):
            begin_time = rospy.get_time()
            print('Hand Eye cnt : ',self.handEyeFeedback.cnt)

            # cv2.waitKey(1) & 0xFF == ord('q')
            # #ipdb.set_trace()()
            if self.handEyeFeedback.r > thred:
                print('Hand Eye redian : ',self.handEyeFeedback.r)
                #ipdb.set_trace()()

                thed_over_cnt += 1

            tracking_try_count += 1
            H_isOK = self.check_if_H_isOK()
            W_isOK = self.check_if_W_isOK()

            h_err = self.handeye_h/2 - self.handEyeFeedback.h 
            w_err = self.handeye_w/2 - self.handEyeFeedback.w 

            if(not H_isOK):
                print(h_err)
                # current_joint5_state = self.pitch_joint5.get_current_pos() 
                if (h_err > 100):
                    h_err = 100
                elif (h_err < -100):
                    h_err = -100
                current_joint5_state = current_joint5_state+(-h_err*0.0004)
                out_h_control = current_joint5_state
                self.pitch_joint5.move_to_goal(out_h_control)

            if(not W_isOK):
                print(w_err)
                current_joint0_state = self.roll_joint0.get_current_pos() 
                if (w_err > 100):
                    w_err = 100
                elif (w_err < -100):
                    w_err = -100
                
                out_w_control = current_joint0_state+(w_err*0.0003)
                self.roll_joint0.move_to_goal(out_w_control)


            #### THIS IS JOG !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #######################################################
            # if(math.fabs(h_err) < 60 and math.fabs(w_err) < 60 and cnt_delta < 0.03):
            #     print('Z-adjust') 
            #     # #ipdb.set_trace()()
            #     delta = 0.007
            #     pos = self.eef_jog.transform([0,0,delta])
            #     ori = arm_orientation

            #     for i in range(5):
            #         print ('calculate IK*****')
            #         angles = self.conbe_ik.calculate(pos,ori)
            #         if(angles == None):
            #             print('calculation wasnot succeed')
            #             # Arm.publish_state('C_FAILED')
            #         else:
            #             self.conbe_arm.move(angles)
            #             break
            #     cnt_delta += delta


            print('***tracking count over ', tracking_try_count)
            finish_time = rospy.get_time()
            take_time = finish_time - begin_time
            print ('take time : {} sec '.format(finish_time - begin_time))
            
            #loop sleep if take time less
            LOOP_TIME = 0.05
            if(take_time < LOOP_TIME):
                rospy.sleep(LOOP_TIME - take_time)
            
            CNT_TRACK_SEC = 5/LOOP_TIME
            if(tracking_try_count > CNT_TRACK_SEC):
                print('***tracking count over ', tracking_try_count)
                self.go_to_ready()
                self.state = 'T_FAILED'
                return self.state
        
        #ipdb.set_trace()()
        #grip tomato
        print('*****grab-tomato')
        rospy.sleep(2.0)
        # self.eef_close()
        # rospy.sleep(0.5)
        self.eef_close()
        rospy.sleep(4.0)

        print('grab correctly')
        self.go_to_box()
        rospy.sleep(2.0)  

        self.eef_open()
        rospy.sleep(2.0)
        self.go_to_ready()
        rospy.sleep(0.5)
        self.go_to_ready()
        rospy.sleep(2.0)
        self.state = ('SUCCEED')
        return self.state




