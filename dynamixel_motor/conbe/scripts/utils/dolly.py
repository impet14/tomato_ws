#!/usr/bin/env python
import rospy
import rosparam
import numpy as np
from math import pi
import math
import tf
import cv2
import geometry_msgs.msg
import target_sub
from std_msgs.msg   import Float64,UInt16

class Dolly_control():
    def __init__(self,rane_distance,moveDirection):
        self._sub_dist = '/Dolly/state'
        self._dist = 0.0  ##tatal distance
        self._local_offset = 0.0
        self._offset = 0
        self._sub = rospy.Subscriber(self._sub_dist, Float64, self.callback,queue_size=1)
        self._pub = rospy.Publisher("Dolly/command", UInt16, queue_size = 1)
        self._move_dist = 0.28
        self.moveDirection = moveDirection
        self.target_marker_subL = target_sub.target_subscriber('L')
        if(moveDirection == 'R'):
            self.left_thred = -0.01
            self.right_thred = rane_distance - 0.15
        else:
            self.left_thred = -rane_distance + 0.15
            self.right_thred = 0.01


    #set current distance as a offset
    def set_offset(self):
        self._local_offset = self._dist
    
    #############################
    ## to get local distance , 
    ## reference from offset
    ## and get local_state()
    #############################
    def get_local_state(self):
        print('dist         : ',self._dist)
        print('local_offset : ',self._local_offset)
        return self.calibrated_value(self._dist - self._local_offset)
        
    def get_state(self):
        return self.calibrated_value(self._dist)
    
    def calibrated_value(self,dist):
        #calibrate actual fb usung bias by experimental results
        return dist * 1.212
        
    def callback(self,msg):
        if(self._offset == 0):
            ###use first feedback as a offset
            self._offset = self._dist
        self._dist = msg.data - self._offset


    def key_control(self):
        background = np.zeros((200,300,3), np.uint8)
        cv2.namedWindow('input_test')

        while not rospy.is_shutdown():
            try: 
                cv2.imshow('input_test',background)
                print('Reference distance:  ',self.get_state())
                input = cv2.waitKey(0)
                if input == ord('r'):  # if key 'z' is pressed 
                    print('r-pressed: Go to right')
                    self._pub.publish(59)
                elif input == ord('l'):  # if key 'x' is pressed 
                    print('l-pressed: Go to left')
                    self._pub.publish(39)
                elif input == ord('q'): # break
                    cv2.destroyAllWindows()
                    break
                elif input == ord('s'):
                    print('stop')
                    self._pub.publish(49)
                print('Reference distance:  ',self.get_state())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def set_dist(self,dist):
        self._move_dist = dist

    def move_dolly(self):
        thred = 0.05
        thred_over_cnt = 0
        self._pub.publish(49)
        while(thred_over_cnt < 3):
            print('thred_over_cnt',thred_over_cnt)

            ######################
            _,py,_ = self.target_marker_subL.getXYZ()

            current_distance = self.get_state()
            print('current distance: ',current_distance,'[m]')

            #change direction when over limitation
            if(current_distance < self.left_thred):
                self.moveDirection = 'R'
            elif(current_distance > self.right_thred):
                self.moveDirection = 'L'
            print('move to ',self.moveDirection)

            #camera recognize tomato
            if(py != -1):
                d_err = -py
                if(self.moveDirection == 'L'):
                    #so as not to go right close to right thredshold
                    if(py < -0.10):
                        print('so as not to go right close to right thredshold')
                        d_err = -1
            #in case the camera doesn't recognize tomato
            else:
                if(self.moveDirection == 'L'):
                    d_err = -1
                    #so as not to go left close to left thredshold
                    if(py > 0.05):
                        print('****so as not to go left close to left thredshold')
                        d_err = 1
                else:
                    d_err = 1
            print('d_err : ',d_err)
            
            ###########################################
            ## usual stated_err
            ###########################################
            D_isOK = bool(math.fabs(d_err) < thred)
            if(D_isOK):
                print('stop')
                thred_over_cnt  += 1
                speed_control = 49
                self._pub.publish(49)
                # rospy.sleep(0.2)

            else:
                if (d_err > 0.7):
                    d_err = 0.7
                elif (d_err < -0.7):
                    d_err = -0.7

                speed_control = 49 + 35 * d_err  #20 before
                
                self._pub.publish(UInt16(speed_control))
            rospy.sleep(0.15)
        return self.moveDirection

    def move_dolly_distance(self):

        self._pub.publish(49)

        current_distance = self.get_state()
        print('current distance: ',current_distance,'[m]')

        # in case that in last time, the goal was inside the range, 
        # but stopped the place in slightly over the range, this should change direction 
        if(current_distance < self.left_thred):
            self.moveDirection = 'R'
        elif(current_distance > self.right_thred):
            self.moveDirection = 'L'

        target_distance = current_distance - self._move_dist if(self.moveDirection == 'L') else current_distance + self._move_dist

        next_moveDirection = self.moveDirection
        
        if(target_distance < self.left_thred):
            target_distance = self.left_thred
            next_moveDirection = 'R'
        elif(target_distance > self.right_thred):
            target_distance = self.right_thred
            next_moveDirection = 'L'

        print('target distance: ',target_distance,'[m]')
        print('move to ',self.moveDirection)

        while(True):
            current_distance = self.get_state()
            if(self.moveDirection == 'L'):
                d_err = -0.5
                if(current_distance < target_distance):
                    break
            else:
                d_err = 0.5
                if(current_distance > target_distance):
                    break

            speed_control = 49 + 22 * d_err 
            self._pub.publish(UInt16(speed_control))
            rospy.sleep(0.05)

        self.moveDirection = next_moveDirection
        return self.moveDirection

    def move_dolly_by_fb():
        thred = 0.03
        thred_over_cnt = 0
        target_y = 1
        ##########IMPORTANT#############
        #### initialize the offset of distance 
        #### to check get reference movement in this loop
        while(thred_over_cnt < 5):
            print('Reference distance:  ',self._sub.get_local_state())
            if(target_y == 1):
                ############################
                #get target-msg only 1time
                px,py,pz = self.target_marker_subL.get()
                if(py != -1):
                    target_y = py
            print('target_y:  ',target_y)
            D_isOK = bool(math.fabs(target_y + self._sub.get_local_state()) < thred)
            if(D_isOK):
                print('stop')
                thred_over_cnt  += 1
                # speed_control = 49
                self.f_pub.publish(49)
            else:
                d_err = -(target_y + self._sub.get_local_state() )
                print(d_err)
                if (d_err > 0.6):
                    d_err = 0.6
                elif (d_err < -0.6):
                    d_err = -0.6
                speed_control = 49 + 22 * d_err 
                self._pub.publish(UInt16(speed_control))
            rospy.sleep(0.15)