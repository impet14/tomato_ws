#!/usr/bin/env python

import rospy
import rosparam
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList
from math import pi
import time

rospy.init_node('State_Publisher')

# Name of the joints in the robotic arm
# Name of motor ID [1,2,3,4,5,6] is specified in the list.
joints = ["Rjoint0","Rjoint1","Rjoint2","Rjoint3","Rjoint4","Rjoint5","Rjoint6"]

#Dynamixel Motors will not be in Position 0 when switched ON. Dynamixel motors will have some initial position value. 
#In the URDF it is assumed that initial joint value is 0 radian. 
#Hence offset is required to match the URDF and the real robot.
#Offset for motor id [10,11,13,14,15,16,17] is specified in the list.
start_ID = 10
# offset = [512,512,512,512,512,512,512]
j0_init = rosparam.get_param("/RArm/joint0_controller/motor/init")
j1_init = rosparam.get_param("/RArm/joint1_controller/motor_master/init")
j2_init = rosparam.get_param("/RArm/joint2_controller/motor/init")
j3_init = rosparam.get_param("/RArm/joint3_controller/motor/init")
j4_init = rosparam.get_param("/RArm/joint4_controller/motor/init")
j5_init = rosparam.get_param("/RArm/joint5_controller/motor/init")
j6_init = rosparam.get_param("/RArm/joint6_controller/motor/init")

offset = [j0_init,j1_init,j2_init,j3_init,j4_init,j5_init,j6_init]
print('R: ',offset)

# offset = [512,2047,512,512,512,512,512]

'''
Function: process(). Callback for subscriber of raw data from dynamixel motor. 
Logic: Dynamixel position = 0 for 0 degree and Dynamixel position = 1023 for 300 degree.
       Current position can be calculated by (position*(300.0/1023))*(pi/180) radian.
       Where position = feddback-offset.
'''
def process(msg):
	joint_states = JointState()
	joint_states.header.stamp = rospy.Time.now()

	for x in msg.motor_states:

		if(x.id != 12):
			# print("ID: {}".format(x.id))
			if(x.id >= 13):
				if(x.id == 17): # in rrbot.xacro, the eef joint's name is left/right _gripper_joint. so chenge the name ans send the same parameter respectively.
					eef_joints = ['Rleft_gripper_joint','Rright_gripper_joint']
					for joint in eef_joints:
						joint_states.name.append(joint)
						joint_states.position.append((x.position-offset[x.id-start_ID-1])*(300.0/1023)*(pi/180))				
				else:
					#in case x.id == 13,14,15,16
					joint_states.name.append(joints[x.id-start_ID-1])
					joint_states.position.append((x.position-offset[x.id-start_ID-1])*(300.0/1023)*(pi/180))
			else:
				if(x.id == 10):
					resolution = 1023
					deg_range  = 300.0
				else:
					#in case x.id == 11
					resolution = 1023 if(offset[1] < 2000) else 4095 if(offset[1] > 2000) else 0
					deg_range  = 300.0 if(offset[1] < 2000) else 360.0

				if (resolution==0): 
					print('please check the offset value of pkg:dynamixelmotor node:state_publisher.py')
					break
				joint_states.name.append(joints[x.id-start_ID])
				joint_states.position.append((x.position-offset[x.id-start_ID])*(deg_range/resolution)*(pi/180))

		#joint_states.velocity.append(x.velocity)
	
	pub.publish(joint_states)

# Subscriber for raw feedback from dynamixel motor. Position of the motor will be in the range of (0,1023).
sub = rospy.Subscriber('/RArm/motor_states/conbe_R_port',MotorStateList,process)
# Publisher for the current position of dynamixel motor in radian
pub = rospy.Publisher('/RArm/joint_states',JointState,queue_size=10)

rospy.spin()