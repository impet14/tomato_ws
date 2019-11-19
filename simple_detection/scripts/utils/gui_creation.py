#!/usr/bin/env python
import rospy
import numpy as np
import cv2

class GUI():
    def __init__(self,img_name,type_name,param_name_list,max_param,MIN,MAX):
        self.img_name = str(img_name)
        self.type_name = str(type_name)
        self.argnum = len(param_name_list)
        self.param_name_list = param_name_list
        self.max_param = max_param
        self.min = MIN
        self.max = MAX
    
    def get_param_as_tuple(self):
        return tuple(self.min),tuple(self.max)
    
    def changeColor(self,val):
        for i in range(self.argnum):
            self.min[i] = int(cv2.getTrackbarPos(self.type_name + self.param_name_list[i] + '_min', self.img_name))
            self.max[i] = int(cv2.getTrackbarPos(self.type_name + self.param_name_list[i] + '_max', self.img_name))
        self.save_track_parameter()

    def create_trackbar(self):
        background = np.zeros((10,300,3), np.uint8)
        cv2.namedWindow(self.img_name)
        #create trackbar
        for i in range(self.argnum):
            cv2.createTrackbar(self.type_name + self.param_name_list[i] + '_min', self.img_name, self.min[i], self.max_param[i], self.changeColor)
            cv2.createTrackbar(self.type_name + self.param_name_list[i] + '_max', self.img_name, self.max[i], self.max_param[i], self.changeColor)
        cv2.imshow(self.img_name,background)
    
    def save_track_parameter(self):
        np.save('./' + self.img_name + self.type_name + 'min.npy',self.min)
        np.save('./' + self.img_name + self.type_name + 'max.npy',self.max)