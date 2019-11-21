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

#for subscripe data of rvis slidebar
import subscribe_rvis_data as rvis_data


class Joint:
    def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.init_node('joint_position_tester')
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')


    def move(self, angles):
            # 
            goal = FollowJointTrajectoryGoal()

        #     char = self.name[0] #either 'f' or 'b'

        #     goal.trajectory.joint_names = ['j1_single_motor', 'j2_dual_motor', 'j3_single_motor', 'j4_single_motor', 'j5_single_motor','j6_single_motor', 'j7_single_motor']
            goal.trajectory.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4','joint5']
            point = JointTrajectoryPoint()
            apply_angles = list(angles)

            #since the joint1-1 & joint1-2 have to move to the same direction but the rotational direction is opposite, 
            #copy the value for id:11 and insert the value * -1 for the id:12
            # apply_angles.insert(2,-1*angles[1])

            point.positions = apply_angles
            print('apply_angles      :   ',apply_angles)
            print('current positions:    ',point.positions)

            point.time_from_start = rospy.Duration(3)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)