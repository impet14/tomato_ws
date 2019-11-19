#!/usr/bin/python
# coding: UTF-8
import rospy
from std_msgs.msg import Int32MultiArray

class handeye():
    def __init__(self,width,height):
        self.width   = width
        self.height  = height
        # self.msg     = Int32MultiArray()
        # self.msg.data   = [width/2, height/2, 0]
        self.handeye_sub = rospy.Subscriber('/usb_cam/handeye_msg', Int32MultiArray, self.callback,queue_size=1)
        self.w   = 0
        self.h   = 0
        self.num = 0
        # rospy.init_node('conbe', anonymous=True)

    def callback(self,msg): 
        self.w   = msg.data[0]
        self.h   = msg.data[1]
        self.num = msg.data[2]
        # self.msg = msg
        # print('subscribe w=%d h=%d num=%d'%(self.w, self.h, self.num))