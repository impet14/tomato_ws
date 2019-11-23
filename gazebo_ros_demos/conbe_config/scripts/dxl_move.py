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
        self._goal_pos = 0
        self._error = 0
    
    def set_goal_pos(self,goal):
        self._goal_pos = goal

    def move(self):
        self.pub.publish(Float64(self._goal_pos))
        rospy.sleep(0.5)
        count = 0
        while (fabs(self._error) > 0.003):
            count += 1
            print('error--', self._error)
            rospy.sleep(0.8)
            if(not self._is_moving or count > 20):
                break
    
    def move_to_goal(self,goal):
        self.set_goal_pos(goal)
        self.move()
    
    def move_with_delta(self,delta):
        self.set_goal_pos(self._current_pos + delta)
        self.move()

    def callback(self,data):
        self._target_pos  = data.goal_pos
        self._current_pos = data.current_pos
        self._error       = data.error
        self._velocity       = data.velocity
        self._is_moving   = data.is_moving
        self._load        = data.load
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