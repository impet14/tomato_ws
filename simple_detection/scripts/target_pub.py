#!/usr/bin/env python
import threading
import sys
import message_filters ### ADD THIS
import rospy
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import roslib; roslib.load_manifest('easy_markers')
from easy_markers.generator import *
import pyrealsense2 as rs
import argparse
import datetime
import sys
from geometry_msgs.msg import PointStamped,Pose

#get intricsic of cam manually
############################################
## 3ch image-shape => ((width,height,3))
############################################

#use rs_rgbd_high.launch
# camera_resolution = {   'rgb-w':1280,   'rgb-h':720,
#                        'depth-w':1280,'depth-h':720 }
#use rs_rgbd_w360.launch
camera_resolution = {   'rgb-w':640,   'rgb-h':360,
                       'depth-w':640,'depth-h':360 }

rs_image_rgb = np.zeros((camera_resolution['rgb-w'],camera_resolution['rgb-h'],3), np.uint8)
rs_image_depth = np.zeros((camera_resolution['depth-w'],camera_resolution['depth-h'],3), np.uint16)

ppx = ppy = fx = fy = 0
setting_isOK = False

if(camera_resolution['depth-w'] == 1280 and camera_resolution['depth-h'] == 720):
    setting_isOK = True
    ppx = 649.253
    ppy = 364.768
    fx  = 927.559
    fy  = 927.358
if(camera_resolution['depth-w'] == 640  and camera_resolution['depth-h'] == 360):
    setting_isOK = True
    ppx = 321.3486328125
    ppy = 179.70550537109375
    fx  = 461.9239807128906
    fy  = 462.3507080078125

if(not setting_isOK):
    print('*** Please set the correct instricsic')
    sys.exit()

#get them from realsense api
depth_scale = 0.0010000000474974513 #seems to be same in any resolution
depth_intrin = rs.intrinsics()
depth_intrin.width = rs_image_rgb.shape[1]
depth_intrin.height = rs_image_rgb.shape[0]
depth_intrin.coeffs = [0, 0, 0, 0, 0]
depth_intrin.ppx = ppx
depth_intrin.ppy = ppy
depth_intrin.fx = fx
depth_intrin.fy = fy
depth_intrin.model = rs.distortion.brown_conrady

#target detect args
enable_image_show = True
enable_fps_show   = True

#------realsense camera class
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
            self.bgr_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.rgb_image = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            print(e)
    def depth_callback(self, data):
        # print(data.encoding) #16uc1
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

#------GUI class to create trackbar class
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

    def create_trackbar(self):
        cv2.namedWindow(self.img_name, cv2.WINDOW_AUTOSIZE)
        #create trackbar
        for i in range(self.argnum):
            cv2.createTrackbar(self.type_name + self.param_name_list[i] + '_min', self.img_name, self.min[i], self.max_param[i], self.changeColor)
            cv2.createTrackbar(self.type_name + self.param_name_list[i] + '_max', self.img_name, self.max[i], self.max_param[i], self.changeColor)


def applyFilter(img,Lmask_MIN,Lmask_MAX,Umask_MIN,Umask_MAX):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        ## Gen lower mask (0-5) and upper mask (175-180) of RED
        mask1 = cv2.inRange(img_hsv, Lmask_MIN, Lmask_MAX)
        mask2 = cv2.inRange(img_hsv, Umask_MIN, Umask_MAX)

        ## Merge the mask and extract the red regions
        redmask = cv2.bitwise_or(mask1, mask2 )
        croped = cv2.bitwise_and(img, img, mask=redmask)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (6,6))
        redmask = cv2.erode(redmask, kernel, iterations = 1)
        redmask = cv2.dilate(redmask, kernel, iterations = 1)
        red_croped = cv2.bitwise_and(croped, croped, mask=redmask)

        return red_croped,croped

def main():

    #Marker Publisher Initialization
    rsProcess = rs_process(camera_resolution)
    rospy.init_node('hand_tracking', anonymous=True)
    rospy.loginfo("--- Target Detection Start!")

    #Marker Publisher Initialize
    pub=rospy.Publisher('/target_marker', Marker,queue_size=1)
    hand_mark = MarkerGenerator()
    hand_mark.type = Marker.SPHERE_LIST
    hand_mark.scale = [.03]*3
    hand_mark.frame_id = '/camera_color_optical_frame'

    #time for fps calculation
    start_time = datetime.datetime.now()
    num_frames = 0

    #define the initial HSV param for mask1&2
    Lmask_MIN = np.array([0,50,20])
    Lmask_MAX = np.array([0,255,255])
    Umask_MIN = np.array([109,160,104])
    Umask_MAX = np.array([161,255,206])
    #the limit of H-channel is 180, others are 255
    hsv_max_param = np.array([180,255,255])
    hsv_param_name_list = ['H','S','V']

    #make instance to create trackbar
    trackbar_mask1 = GUI('SETTING','mask1--',hsv_param_name_list,hsv_max_param,Lmask_MIN,Lmask_MAX)
    trackbar_mask2 = GUI('SETTING','mask2--',hsv_param_name_list,hsv_max_param,Umask_MIN,Umask_MAX)
    trackbar_mask1.create_trackbar()
    trackbar_mask2.create_trackbar()

    #define the initial depth param in CentiMeters
    depth_MIN = np.array([47])
    depth_MAX = np.array([100])
    #don't get the depth more than this
    depth_max_param = np.array([500])

    depth_param_name_list = ['DEPTH']
    trackbar_depth = GUI('SETTING','dist[cm]',depth_param_name_list,depth_max_param,depth_MIN,depth_MAX)
    trackbar_depth.create_trackbar()

    while not rospy.is_shutdown():
        #get rgb,depth frames for synchronized frames
        if not rsProcess.isOK:
            continue

        rsProcess.isOK = False

        #start processiog
        img_bgr = rsProcess.bgr_image
        img_rgb = rsProcess.rgb_image
        img_depth = rsProcess.depth_image


        ##Sometimes the depth.shape could be ((height,width,chnnel)) which are swapped(height<->width)
        if(not (img_bgr.shape[0] == img_depth.shape[0])):
            print('*******************************************************************')
            print('**** THE CAMERA INFO IS INVALID **** restart the camera launch file' )
            sys.exit()

        depth_map = cv2.applyColorMap(cv2.convertScaleAbs(rs_image_depth, alpha=0.03), cv2.COLORMAP_JET)

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 100
        depth_image_3d = np.dstack((img_depth,img_depth,img_depth)) #depth image is 1 channel, color is 3 channels

        clipping_distance_in_centimeters = trackbar_depth.get_param_as_tuple()

        min_clipping_distance_in_meters = float(clipping_distance_in_centimeters[0][0]) / 100.0  #1 meter
        max_clipping_distance_in_meters = float(clipping_distance_in_centimeters[1][0]) / 100.0  #1 meter


        min_clipping_distance = min_clipping_distance_in_meters / depth_scale
        max_clipping_distance = max_clipping_distance_in_meters / depth_scale

        bg_removed_max = np.where((depth_image_3d > max_clipping_distance) | (depth_image_3d <= 0), grey_color, img_bgr)
        bg_removed = np.where((depth_image_3d < min_clipping_distance) | (depth_image_3d <= 0), grey_color, bg_removed_max)

        #apply filter to extract the red color
        Lmask_MIN, Lmask_MAX = trackbar_mask1.get_param_as_tuple()
        Umask_MIN, Umask_MAX = trackbar_mask2.get_param_as_tuple()

        gray_croped,croped = applyFilter(bg_removed,Lmask_MIN,Lmask_MAX,Umask_MIN,Umask_MAX)

        # print(gray_croped.shape)
        box_index = np.where(gray_croped != 0)

        if len(box_index[0]) >= 10 : # check red exists

            left = np.amin(box_index[1])
            right = np.amax(box_index[1])
            top = np.amin(box_index[0])
            bottom = np.amax(box_index[0])

            i_centor = int(left + right) / 2
            j_centor = int(top + bottom) / 2

            box_img = croped[left:right,top:bottom]

            #add l r t b
            print ("detected red")
            align_img_target_RGB = img_rgb[int(top):int(bottom), int(left):int(right)]
            align_img_depth      = depth_map[int(top):int(bottom),int(left):int(right)]
            align_target_detect = np.hstack((align_img_target_RGB, align_img_depth))

            #display window
            if (enable_image_show):
                cv2.namedWindow('gray_croped', cv2.WINDOW_AUTOSIZE)
                cv2.imshow("gray_croped", gray_croped)
                cv2.namedWindow('align target', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('align target', align_target_detect)

            depth_pixel = [(int(left)+int(right))/2, (int(top)+int(bottom))/2]
            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, img_depth[depth_pixel[1],depth_pixel[0]]*depth_scale)

            #target mark
            hand_mark.counter = 0
            t = rospy.get_time()
            hand_mark.color = [1,0,0,1]
            m = hand_mark.marker(points= [(depth_point[0], depth_point[1], depth_point[2])])
            pub.publish(m)

            cv2.circle(bg_removed, (i_centor, j_centor), radius=7, color=(255,0,0), thickness=-1)

        # Calculate Frames per second (FPS)
        num_frames += 1
        elapsed_time = (datetime.datetime.now() -
                        start_time).total_seconds()
        fps = num_frames / elapsed_time

        #display window
        if (enable_image_show):
            # Display FPS on frame
            if (enable_fps_show):
                cv2.putText(bg_removed, str(fps), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
            cv2.imshow("bg_removed_RGB", cv2.cvtColor(bg_removed, cv2.COLOR_BGR2RGB))
        else:
            if (enable_fps_show):
                print("frames processed: ",  num_frames, "elapsed time: ", elapsed_time, "fps: ", str(int(fps)))

        if cv2.waitKey(10) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

if __name__ == '__main__':
    main()
    print("Close Target Detection Publisher")
    # sess.close()
