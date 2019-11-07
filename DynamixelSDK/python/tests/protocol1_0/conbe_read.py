#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library


class DXL_READ():
    def __init__(self,id,motorType):
        # Control table address
        self.ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model (in case of DX-117/MX-64AR, these params are same)
        self.ADDR_MX_GOAL_POSITION      = 30               # http://emanual.robotis.com/docs/en/dxl/dx/dx-117/
        self.ADDR_MX_PRESENT_POSITION   = 36               # http://emanual.robotis.com/docs/en/dxl/mx/mx-64/

        # Protocol version
        self.PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID                      = id                # Dynamixel ID 
        self.BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
        self.DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        #DX-117 resolution : 0.29[deg] = 1024 [pulse/rev]
        # Angle : 300
        # so it varies from 0 to 1023

        #MX-64AR resolution : 4096 [pulse/rev]
        # Angle : 360
        # so it varies from o to 4095. 
        MAX_POSITION_VALUE = {'DX117':1023,'MX64AR':4095}

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE  = MAX_POSITION_VALUE[motorType]            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        self.index = 0
        self.dxl_goal_position = [self.DXL_MINIMUM_POSITION_VALUE, self.DXL_MAXIMUM_POSITION_VALUE]         # Goal position

    def init_Handler(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def  set_port_baudrate(self):
        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def enable_Torque(self):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
        # dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MX_TORQUE_DISABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

    def read_data(self):
        # while 1:
        #     print("Press any key to continue! (or press ESC to quit!)")
        #     if getch() == chr(0x1b):
        #         break
            # Write goal position
            # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
            # if dxl_comm_result != COMM_SUCCESS:
            #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            # elif dxl_error != 0:
            #     print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (self.DXL_ID, self.dxl_goal_position[self.index], dxl_present_position))

        if not abs(self.dxl_goal_position[self.index] - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD:
            print('calculation was finished')
            # break

            # # Change goal position
            # if index == 0:
            #     index = 1
            # else:
            #     index = 0

    def disable_torque(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # Close port
        self.portHandler.closePort()

    def init_setup(self):
        self.init_Handler()
        self.set_port_baudrate()
        # self.enable_Torque()
        ## after this func, you can call 
        ## self.read_data() -> then -> self.disable_torque()

if __name__ == "__main__":
    #[1]make instance for each motor
    M11 = DXL_READ(id=11,motorType='DX117')
    M11.init_setup()
    M12 = DXL_READ(id=12,motorType='DX117')
    M12.init_setup()

    #[2]read motor angle 
    while(1):
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break
        for i in range(100):
            M11.read_data()
            M12.read_data()

    #[3]disable torque and close port
    M11.disable_torque()
    M12.disable_torque()