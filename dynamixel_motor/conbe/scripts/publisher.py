#!/usr/bin/env python
# import roslib;roslib.load_manifest('dynamixel_tutorials')
import roslib;roslib.load_manifest('conbe')
import rospy
from std_msgs.msg import Float64
import time


#need to get current position of moter in radians

if __name__ =="__main__":
    rospy.init_node("Publisher")
    pub = rospy.Publisher('/joint2_controller/command', Float64)
    r = rospy.Rate(10)

    def dxl_move(start,stop):
        if(start < stop):
            for deg in range (start,stop):
                print('control : ',deg, '[deg]')
                int = deg*3.14/180  #Converting degrees to radians.
                pub.publish(int)
                r.sleep()
        else:
            for deg in range(start-stop):
                print('control : ',start-deg, '[deg]')
                int = (start-deg)*3.14/180  #Converting degrees to radians.
                pub.publish(int)
                r.sleep()

    def dxl_move_goal(deg):
        print('control : ',deg, '[deg]')
        int = deg*3.14/180  #Converting degrees to radians.
        pub.publish(int)
        r.sleep()


    if not rospy.is_shutdown():
        # dxl_move(0,10)
        # time.sleep(5)
        # dxl_move(45,-45)
        # dxl_move(-45,45)

        dxl_move_goal(0)
        time.sleep(3)
        dxl_move_goal(90)
        time.sleep(3)
        dxl_move_goal(-90)
        time.sleep(3)
        dxl_move_goal(0)

    #     counter = -90
    #     i=0
    #     while (True):
    #         int = counter*3.14/180  #Converting degrees to radians.
    #         pub.publish(int)
    # #This part ensures that the input is sent continously clockwise and anti-clockwise.
    #         if(counter == 90):
    #             i = 1
    #             break # otherwise infinite roop 
    #         elif(counter == -90):
    #             i=0
    #         if(i==1):
    #             counter-=1
    #         elif(i==0):
    #             counter+=1
    #         r.sleep()