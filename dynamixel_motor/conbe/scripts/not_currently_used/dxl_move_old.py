#!/usr/bin/env python

import rospy
from math import fabs
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

class DXL_CONTROL():
    def __init__(self,node_name,control_joint,max_load):
        self.pub_dist = '/' + control_joint + '/command'
        self.sub_dist = '/' + control_joint + '/state'
        self.node_name = node_name
        self.pub = rospy.Publisher(self.pub_dist,Float64,queue_size=10)
        self.goal_pos = 0
        self.max_load = max_load
        self._error = 100
        self._is_moving = True
        rospy.init_node(self.node_name, anonymous=True)
        rospy.Subscriber(self.sub_dist, JointState, self.transform_callback)

    def transform_callback(self,data):
        rospy.loginfo(rospy.get_name() + ': Target angle {0}'.format(self.goal_pos))
        rospy.loginfo(rospy.get_name() + ': Goal         {0} [rad]'.format(data.goal_pos))
        rospy.loginfo(rospy.get_name() + ': position     {0} [rad]'.format(data.current_pos))
        rospy.loginfo(rospy.get_name() + ': velocity     {0} [rad/s]'.format(data.velocity))
        rospy.loginfo(rospy.get_name() + ': Load         {0}'.format(data.load))
        rospy.loginfo(rospy.get_name() + ': Moving       {0}'.format(data.is_moving))
        rospy.loginfo(rospy.get_name() + ': Error        {0}'.format(data.error))

        self._error = data.error
        self._is_moving = data.is_moving

        # If the motor has reached its limit, publish a new command.
        # if fabs(data.load) > self.max_load:
            # self.goal_pos = data.current_pos
            # self.pub.publish(Float64(self.goal_pos))

        #     str = "goal pos was changed: Time: {0} Moving motor to {1}" .format(rospy.get_time(), self.goal_pos)
        #     rospy.loginfo(str)

        # str = "reached to GOAL: Time: {0} Moving motor to {1}" .format(rospy.get_time(), self.goal_pos)
        # rospy.loginfo(str)
            

    def open(self):
        self.goal_pos = 1.5
        # Initial movement.
        self.pub.publish(Float64(self.goal_pos))
        while (fabs(self._error) > 0.003):
            print('error--', self._error)
            rospy.sleep(0.8)
            if(not self._is_moving):
                break

    def close(self):
        self.goal_pos = -1.5
        # Initial movement.
        self.pub.publish(Float64(self.goal_pos))
        while (fabs(self._error) > 0.003):
            print('error--', self._error)
            rospy.sleep(0.8)
            if(not self._is_moving):
                break


if __name__ == '__main__':
    try:
        eef_joint = DXL_CONTROL(control_joint='joint6_controller',max_load=0.006)
        eef_joint.open()
        eef_joint.close()

    except rospy.ROSInterruptException:
        pass