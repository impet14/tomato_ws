]#!/usr/bin/env python

import rospy
import numpy as np
from math import fabs
from sensor_msgs.msg import JointState as JointState_rvis
from std_msgs.msg import Float64

#for client trajectry control
import client_trajectory

class ReadRvis():
    def __init__(self,topic_name):
        self.sub_rvis_dist = '/' + topic_name
        self.prev_position = np.zeros(10)
        self.arm = client_trajectory.Joint('/conbe_controller')
        # rospy.init_node('simulator_control', anonymous=True)
        self.arm.move([0.0,0.0,0.0,0.0,0.0,0.0]) #use only 0~5
        self.sub =  rospy.Subscriber('/joint_states', JointState_rvis, self.callback,queue_size=10)
        self.r  = rospy.Rate(20)
        rospy.spin()

        
    def callback(self,data):
        # print(data)
        for i in range(6):
            COMMAND_FLAG = False
            if(fabs(self.prev_position[i]-data.position[i]) > 0.01):
                print(data)
                rospy.loginfo(rospy.get_name() + ': name       {0}      '.format(data.name[i]))
                rospy.loginfo(rospy.get_name() + ': position   {0} [rad]'.format(data.position[i]))
                COMMAND_FLAG = True
            if(COMMAND_FLAG):
                #move arm with the value in slidebar
                self.arm.move(data.position[:6]) #use only 0~6
            self.prev_position[i] = data.position[i]
        self.r.sleep()

        

if __name__ == '__main__':
 
    rvis_subscriber = ReadRvis('joint_states')