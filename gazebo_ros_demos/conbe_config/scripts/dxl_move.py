#!/usr/bin/env python
import rospy
from math import fabs
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

class DXL_CONTROL():
    def __init__(self,control_joint):
        self.pub_dist = '/' + control_joint + '/command'
        self.pub = rospy.Publisher(self.pub_dist,Float64,queue_size=10)
            
    def open(self):
        self.goal_pos = 1.5
        # Initial movement.
        self.pub.publish(Float64(self.goal_pos))
        rospy.sleep(2)

    def close(self):
        self.goal_pos = -0.5
        # Initial movement.
        self.pub.publish(Float64(self.goal_pos))
        rospy.sleep(2)