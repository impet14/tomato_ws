#!/usr/bin/env python

import rospy
import numpy as np
from math import fabs
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64

#for client trajectry control
import client_trajectory

class ReadMoveit():
    def __init__(self,topic_name):
        self.sub_rvis_dist = '/' + topic_name
        self.prev_position = np.zeros(10)
        self.arm = client_trajectory.Joint('f_arm')
        # rospy.init_node('simulator_control', anonymous=True)
        self.arm.move([0.0,0.0,0.0,0.0,0.0,0.0,0.0]) #use only 0~6
        
    def transform_callback(self,data):
        # print(data)
        for i in range(7):
            COMMAND_FLAG = False
            if(fabs(self.prev_position[i]-data.position[i]) > 0.01):
                print(data)
                rospy.loginfo(rospy.get_name() + ': name       {0}      '.format(data.name[i]))
                rospy.loginfo(rospy.get_name() + ': position   {0} [rad]'.format(data.position[i]))
                COMMAND_FLAG = True
            if(COMMAND_FLAG):
                #move arm with the value in slidebar
                self.arm.move(data.position[:7]) #use only 0~6
            self.prev_position[i] = data.position[i]

    def read(self):
        rospy.Subscriber(self.sub_rvis_dist, JointState_rvis, self.transform_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        moveit_subscriber = ReadRvis('execute_trajectory/goal')
        moveit_subscriber.read()

    except rospy.ROSInterruptException:
        pass