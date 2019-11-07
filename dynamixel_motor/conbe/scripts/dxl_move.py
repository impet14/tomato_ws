#!/usr/bin/env python

import rospy
from math import fabs
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

class DXL_CONTROL():
    def __init__(self,control_joint):
        self.pub_dist = '/' + control_joint + '/command'
        self.sub_dist = '/' + control_joint + '/state'
        self.pub = rospy.Publisher(self.pub_dist,Float64,queue_size=10)
        self.goal_pos = 0
        self.state = True

    def transform_callback(self,data):
        rospy.loginfo(rospy.get_name() + ': Target angle {0}'.format(self.goal_pos))

        rospy.loginfo(rospy.get_name() + ': Goal         {0} [rad]'.format(data.goal_pos))
        rospy.loginfo(rospy.get_name() + ': position     {0} [rad]'.format(data.current_pos))
        rospy.loginfo(rospy.get_name() + ': velocity     {0} [rad/s]'.format(data.velocity))
        rospy.loginfo(rospy.get_name() + ': Load         {0}'.format(data.load))
        rospy.loginfo(rospy.get_name() + ': Moving       {0}'.format(data.is_moving))
        rospy.loginfo(rospy.get_name() + ': Error        {0}'.format(data.error))

        # If the motor has reached its limit, publish a new command.
        if fabs(self.goal_pos-data.current_pos) < 0.01:
            # if self.goal_pos == 0:
            #     self.goal_pos = 1.57
            # else:
            #     self.goal_pos = 0

            str = "Time: {0} Moving motor to {1}" .format(rospy.get_time(), self.goal_pos)
            rospy.loginfo(str)

            # self.pub.publish(Float64(self.goal_pos))

    def control(self):
        self.state = True
        rospy.init_node('dxl_control', anonymous=True)
        rospy.Subscriber(self.sub_dist, JointState, self.transform_callback)
        # Initial movement.
        self.pub.publish(Float64(self.goal_pos))
        rospy.spin()


if __name__ == '__main__':
    try:
        # dxl_j1 = DXL_CONTROL('joint1_controller')
        # dxl_j1.goal_pos = 1.5
        # dxl_j1.control()

        # dxl_j2 = DXL_CONTROL('f_arm_controller')
        # dxl_j2.goal_pos = ([0.0,0.0,0.0,0.0,0.0,0.0,])
        # dxl_j2.control()

        # print('3')
        # dxl_j3 = DXL_CONTROL('joint3_controller')
        # dxl_j3.goal_pos = 0
        # dxl_j3.control()

        # print('4')
        # dxl_j4 = DXL_CONTROL('joint4_controller')
        # dxl_j4.goal_pos = 0
        # dxl_j4.control()

        # print('5')
        # dxl_j5 = DXL_CONTROL('joint5_controller')
        # dxl_j5.goal_pos = 0
        # dxl_j5.control()

        print('6')
        dxl_j6 = DXL_CONTROL('joint6_controller')
        dxl_j6.goal_pos = 1.5
        dxl_j6.control()

        # print('7')
        # dxl_j7 = DXL_CONTROL('joint7_controller')
        # dxl_j7.goal_pos = 0
        # dxl_j7.control()
        # rospy.sleep(5)

        # dxl_control()
    except rospy.ROSInterruptException:
        pass