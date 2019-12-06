#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs

class rs_process:
    def __init__(self,resolution):
        self.bridge = CvBridge()
        image_align_hand_color = "/camera/color/image_rect_color"
        image_align_hand_depth = "/camera/aligned_depth_to_color/image_raw"
        self.image_sub = rospy.Subscriber(image_align_hand_color, Image, self.color_callback, queue_size=1)
        self.image_sub = rospy.Subscriber(image_align_hand_depth, Image, self.depth_callback, queue_size=1)
        self.isOK = False
        self.depth_image = np.zeros((resolution['depth-w'],resolution['depth-h'],3), np.uint16)
        self.rgb_image = np.zeros((resolution['rgb-w'],resolution['rgb-h'],3), np.uint8)
        self.bgr_image = np.zeros((resolution['rgb-w'],resolution['rgb-h'],3), np.uint8)

    def color_callback(self, data):
        self.isOK = True
        # print (data.encoding) #rgb8
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.bgr_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            print(e)
    def depth_callback(self, data):
        # print(data.encoding) #16uc1
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)