#!/usr/bin/env python

import rospy
import numpy as np
from math import fabs
from sensor_msgs.msg import JointState as JointState_rvis
from std_msgs.msg import Float64

class ReadRvis():
    def __init__(self,topic_name):
        self.sub_rvis_dist = '/' + topic_name
        self.prev_position = np.zeros(8)
        # control_state = JointState_rvis()

    def transform_callback(self,data):

        for i in range(8):
            if(fabs(self.prev_position[i]-data.position[i]) > 0.01):
                rospy.loginfo(rospy.get_name() + ': name       {0}      '.format(data.name[i]))
                rospy.loginfo(rospy.get_name() + ': position   {0} [rad]'.format(data.position[i]))
            self.prev_position[i] = data.position[i]

    def read(self):
        rospy.init_node('simulator_control', anonymous=True)
        rospy.Subscriber(self.sub_rvis_dist, JointState_rvis, self.transform_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        rvis_subscriber = ReadRvis('joint_states')
        rvis_subscriber.read()

    except rospy.ROSInterruptException:
        pass