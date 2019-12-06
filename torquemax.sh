#!/bin/bash
rosparam set /LArm/dynamixel/conbe_L_port/10/max_torque 30
rosparam set /LArm/dynamixel/conbe_L_port/11/max_torque 30
rosparam set /LArm/dynamixel/conbe_L_port/12/max_torque 30
rosparam set /LArm/dynamixel/conbe_L_port/13/max_torque 30
rosparam set /LArm/dynamixel/conbe_L_port/14/max_torque 30
rosparam set /LArm/dynamixel/conbe_L_port/15/max_torque 30
rosparam set /LArm/dynamixel/conbe_L_port/16/max_torque 30
rosparam set /LArm/dynamixel/conbe_L_port/17/max_torque 30
echo 'read max_torque'
rosparam get /LArm/dynamixel/conbe_L_port/10/max_torque 
rosparam get /LArm/dynamixel/conbe_L_port/11/max_torque 
rosparam get /LArm/dynamixel/conbe_L_port/12/max_torque 
rosparam get /LArm/dynamixel/conbe_L_port/13/max_torque 
rosparam get /LArm/dynamixel/conbe_L_port/14/max_torque 
rosparam get /LArm/dynamixel/conbe_L_port/15/max_torque 
rosparam get /LArm/dynamixel/conbe_L_port/16/max_torque 
rosparam get /LArm/dynamixel/conbe_L_port/17/max_torque 

rosparam set /LArm/dynamixel/conbe_L_port/10/max_velocity 200
rosparam set /LArm/dynamixel/conbe_L_port/11/max_velocity 200
rosparam set /LArm/dynamixel/conbe_L_port/12/max_velocity 200
rosparam set /LArm/dynamixel/conbe_L_port/13/max_velocity 200
rosparam set /LArm/dynamixel/conbe_L_port/14/max_velocity 200
rosparam set /LArm/dynamixel/conbe_L_port/15/max_velocity 200
rosparam set /LArm/dynamixel/conbe_L_port/16/max_velocity 200
rosparam set /LArm/dynamixel/conbe_L_port/17/max_velocity 200
echo 'read max_velocity'
rosparam get /LArm/dynamixel/conbe_L_port/10/max_velocity 
rosparam get /LArm/dynamixel/conbe_L_port/11/max_velocity 
rosparam get /LArm/dynamixel/conbe_L_port/12/max_velocity 
rosparam get /LArm/dynamixel/conbe_L_port/13/max_velocity 
rosparam get /LArm/dynamixel/conbe_L_port/14/max_velocity 
rosparam get /LArm/dynamixel/conbe_L_port/15/max_velocity 
rosparam get /LArm/dynamixel/conbe_L_port/16/max_velocity 
rosparam get /LArm/dynamixel/conbe_L_port/17/max_velocity 
##

rosparam set /LArm/joint0_controller/joint_speed 25
rosparam set /LArm/joint1_controller/joint_speed 25
rosparam set /LArm/joint2_controller/joint_speed 25
rosparam set /LArm/joint3_controller/joint_speed 25
rosparam set /LArm/joint4_controller/joint_speed 25
rosparam set /LArm/joint5_controller/joint_speed 25
rosparam set /LArm/joint6_controller/joint_speed 25
rosparam set /LArm/joint7_controller/joint_speed 5

echo 'read joint speed'
rosparam get /LArm/joint0_controller/joint_speed 
rosparam get /LArm/joint1_controller/joint_speed 
rosparam get /LArm/joint2_controller/joint_speed 
rosparam get /LArm/joint3_controller/joint_speed 
rosparam get /LArm/joint4_controller/joint_speed 
rosparam get /LArm/joint5_controller/joint_speed 
rosparam get /LArm/joint6_controller/joint_speed 
rosparam get /LArm/joint7_controller/joint_speed 
