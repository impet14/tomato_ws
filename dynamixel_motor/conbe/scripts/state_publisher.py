#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList
from math import pi
import time

rospy.init_node('State_Publisher')

# Name of the joints in the robotic arm
# Name of motor ID [1,2,3,4,5,6] is specified in the list.
joints = ["joint0","joint1","joint2","joint3","joint4","joint5","joint6"]
# joints = ["joint0","joint1-1","joint1-2","joint2","joint3","joint4","joint5","joint6"]

#Dynamixel Motors will not be in Position 0 when switched ON. Dynamixel motors will have some initial position value. 
#In the URDF it is assumed that initial joint value is 0 radian. 
#Hence offset is required to match the URDF and the real robot.
#Offset for motor id [10,11,13,14,15,16,17] is specified in the list.
start_ID = 10
# offset = [512,512,512,512,512,512,512]
offset = [512,2047,512,512,512,512,512]

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
					eef_joints = ['left_gripper_joint','right_gripper_joint']
					for joint in eef_joints:
						joint_states.name.append(joint)
						joint_states.position.append((x.position-offset[x.id-start_ID-1])*(300.0/1023)*(pi/180))				
				else:
					joint_states.name.append(joints[x.id-start_ID-1])
					joint_states.position.append((x.position-offset[x.id-start_ID-1])*(300.0/1023)*(pi/180))
			else:
				resolution = 1023 if(offset[1] == 512) else 4095 if(offset[1] == 2047) else 0
				if (resolution==0): 
					print('please check the offset value of pkg:dynamixelmotor node:state_publisher.py')
					break
				joint_states.name.append(joints[x.id-start_ID])
				joint_states.position.append((x.position-offset[x.id-start_ID])*(300.0/resolution)*(pi/180))

		#joint_states.velocity.append(x.velocity)
	
	pub.publish(joint_states)

# Subscriber for raw feedback from dynamixel motor. Position of the motor will be in the range of (0,1023).
sub = rospy.Subscriber('/motor_states/conbe_L_port',MotorStateList,process)
# Publisher for the current position of dynamixel motor in radian
pub = rospy.Publisher('/conbe/joint_states',JointState,queue_size=10)

rospy.spin()