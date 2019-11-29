###  IMPORTANT
# Even though there are 2 controller for joint2, you can input 1 value for joint2
# Check the code in move function!!!
###

#!/usr/bin/env python
import roslib
roslib.load_manifest('conbe')
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Joint:
    def __init__(self, controller_name):
            #arm_name should be b_arm or f_arm
            self.jta = actionlib.SimpleActionClient(controller_name+'/follow_joint_trajectory', FollowJointTrajectoryAction)
        #     rospy.init_node('joint_position_tester')
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

    def move(self, angles):
            goal = FollowJointTrajectoryGoal()

            goal.trajectory.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4','joint5']
            point = JointTrajectoryPoint()
            apply_angles = list(angles)

            #since the joint1-1 & joint1-2 have to move to the same direction but the rotational direction is opposite, 
            #copy the value for id:11 and insert the value * -1 for the id:12
            # apply_angles.insert(2,-1*angles[1])  => This code in impletemted in the source code

            point.positions = apply_angles
            print('apply_angles      :   ',apply_angles)
            print('current positions:    ',point.positions)

            point.time_from_start = rospy.Duration(3)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)