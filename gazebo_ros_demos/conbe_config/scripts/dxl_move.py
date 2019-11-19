#!/usr/bin/env python
import rospy
from math import fabs
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

class DXL_CONTROL():
    def __init__(self,control_joint):
        self.pub_dist = '/' + control_joint + '/command'
        self.sub_dist = '/' + control_joint + '/state'
        self.pub = rospy.Publisher(self.pub_dist,Float64,queue_size=1)
        self.sub =  rospy.Subscriber(self.sub_dist, JointState, self.callback,queue_size=1)
        self.current_pos = 0 
        self.target_pos = 0
        self.load = 0

    def open(self):
        self.goal_pos = 0.34
        self.pub.publish(Float64(self.goal_pos))
        while (fabs(self.current_pos - self.target_pos) > 0.01):
            rospy.sleep(0.1)

    def close(self):
        self.goal_pos = -1.46
        self.pub.publish(Float64(self.goal_pos))
        while (fabs(self.current_pos - self.target_pos) > 0.01):
            rospy.sleep(0.1)   
    
    def callback(self,data):
        self.target_pos    = data.goal_pos
        self.current_pos = data.current_pos
        self.load        = data.load
        # rospy.loginfo(rospy.get_name() + ': Target angle {0}'.format(self.goal_pos))
        # rospy.loginfo(rospy.get_name() + ': Goal         {0} [rad]'.format(data.goal_pos))
        # rospy.loginfo(rospy.get_name() + ': position     {0} [rad]'.format(data.current_pos))
        # rospy.loginfo(rospy.get_name() + ': velocity     {0} [rad/s]'.format(data.velocity))
        # rospy.loginfo(rospy.get_name() + ': Load         {0}'.format(data.load))
        # rospy.loginfo(rospy.get_name() + ': Moving       {0}'.format(data.is_moving))
        # rospy.loginfo(rospy.get_name() + ': Error        {0}'.format(data.error))
        # If the motor has reached its limit, publish a new command.
        
        # if fabs(data.load) > self.max_load:
        #     self.goal_pos = data.current_pos
        #     self.pub.publish(Float64(self.goal_pos))

        #     str = "goal pos was changed: Time: {0} Moving motor to {1}" .format(rospy.get_time(), self.goal_pos)
        #     rospy.loginfo(str)

        # str = "reached to GOAL: Time: {0} Moving motor to {1}" .format(rospy.get_time(), self.goal_pos)
        # rospy.loginfo(str)