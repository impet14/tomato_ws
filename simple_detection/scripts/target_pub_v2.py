#!/usr/bin/env python
import threading
import sys
import message_filters ### ADD THIS
import rospy
import numpy as np
import math
import glob
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
from utils import rs_process
from utils import gui_creation
import ipdb

#get intricsic of cam manually
############################################
## 3ch image-shape => ((width,height,3))
############################################

#use rs_rgbd_high.launch
# camera_resolution = {   'rgb-w':1280,   'rgb-h':720,
#                        'depth-w':1280,'depth-h':720 }
#use rs_rgbd_w360.launch
depth_w = int(rospy.get_param('/depth_width'))
depth_h = int(rospy.get_param('/depth_height'))
color_w = int(rospy.get_param('/color_width'))
color_h = int(rospy.get_param('/color_height'))
camera_resolution = {   'rgb-w':color_w,   'rgb-h':color_h,
                       'depth-w':depth_w,'depth-h':depth_h }

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

############################
#get them from realsense api
############################
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

    ####K-means to clustering

    return red_croped,croped

def get_colored_area(cv_image,Lmask_MIN,Lmask_MAX,Umask_MIN,Umask_MAX):
    cv2.imshow('cv_image',cv_image)
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    cv2.imshow('hsv_image',hsv_image)
    # for _ in range(3):
    # median_hsv_img = cv2.bilateralFilter(hsv_image, d=9,sigmaColor=50,sigmaSpace=7)

    # median_hsv_img = cv2.medianBlur(hsv_image, 11)
    # median_bgr_img = cv2.cvtColor(median_hsv_img, cv2.COLOR_HSV2BGR)
    median_hsv_img = hsv_image
    median_bgr_img = cv_image
    # cv2.imshow('median_bgr_img',median_bgr_img)

    mask_image1 = cv2.inRange(median_hsv_img, Lmask_MIN, Lmask_MAX)
    mask_image2 = cv2.inRange(median_hsv_img, Umask_MIN, Umask_MAX)
    ## Merge the mask and extract the red regions
    redmask = cv2.bitwise_or(mask_image1, mask_image2 )
    cv2.imshow('redmask',redmask)
    extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=redmask)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2,2))
    redmask = cv2.erode(redmask, kernel, iterations = 1)
    redmask = cv2.dilate(redmask, kernel, iterations = 1)
    median_filtered_extracted_imag = cv2.bitwise_and(median_bgr_img, median_bgr_img, mask=redmask)

    area = cv2.countNonZero(redmask)

    return (area, median_filtered_extracted_imag)

def main():
    rospy.init_node('hand_tracking', anonymous=True)
    rospy.loginfo("--- Target Detection Start!")

    #######################################
    ##make new instance to subscribe image
    #######################################
    rsProcess = rs_process.rs_process(camera_resolution)

    #############################################
    ##Create GUI for hsv mask setting
    #############################################
    #define the initial HSV param for mask1&2
    Lmask_MIN = np.array([0,50,20])
    Lmask_MAX = np.array([0,255,255])
    Umask_MIN = np.array([109,160,104])
    Umask_MAX = np.array([161,255,206])
    #the limit of H-channel is 180, others are 255
    hsv_max_param = np.array([180,255,255])
    hsv_param_name_list = ['H','S','V']
    #load the param
    rs_img_name = 'RS_SETTING'
    type_name1 = 'mask1--'
    type_name2 = 'mask2--'
    if(len(glob.glob('./'+ rs_img_name + 'mask*.npy')) == 4):
        Lmask_MIN = np.load('./' + rs_img_name + type_name1 + 'min.npy')
        Lmask_MAX = np.load('./' + rs_img_name + type_name1 + 'max.npy')
        Umask_MIN = np.load('./' + rs_img_name + type_name2 + 'min.npy')
        Umask_MAX = np.load('./' + rs_img_name + type_name2 + 'max.npy')
    else:
        Lmask_MIN = np.array([0,101,144])
        Lmask_MAX = np.array([7,255,240])
        Umask_MIN = np.array([170,160,104])
        Umask_MAX = np.array([180,255,206])

    #make instance to create trackbar
    trackbar_mask1 = gui_creation.GUI(rs_img_name,type_name1,hsv_param_name_list,hsv_max_param,Lmask_MIN,Lmask_MAX)
    trackbar_mask2 = gui_creation.GUI(rs_img_name,type_name2,hsv_param_name_list,hsv_max_param,Umask_MIN,Umask_MAX)
    trackbar_mask1.create_trackbar()
    trackbar_mask2.create_trackbar()

    #############################################
    ##Create GUI for depth setting
    #############################################
    #define the initial depth param in CentiMeters
    # depth_MIN = np.array([47])
    # depth_MAX = np.array([100])
    #don't get the depth more than this
    depth_max_param = np.array([500])
    depth_param_name_list = ['DEPTH']
    #load the param
    type_name = 'dist[cm]'
    if(len(glob.glob('./'+ rs_img_name+ 'dist*.npy')) == 2):
        depth_MIN = np.load('./' + rs_img_name + type_name + 'min.npy')
        depth_MAX = np.load('./' + rs_img_name + type_name + 'max.npy')
    else:
        depth_MIN = np.array([47])
        depth_MAX = np.array([100])
    trackbar_depth = gui_creation.GUI(rs_img_name,type_name,depth_param_name_list,depth_max_param,depth_MIN,depth_MAX)
    trackbar_depth.create_trackbar()

    #############################################
    #Marker Publisher Initialization
    #############################################
    pub=rospy.Publisher('/target_marker', Marker,queue_size=1)
    hand_mark = MarkerGenerator()
    hand_mark.type = Marker.SPHERE_LIST
    hand_mark.scale = [.03]*3
    hand_mark.frame_id = '/camera_color_optical_frame'
    #time for fps calculation
    start_time = datetime.datetime.now()
    num_frames = 0

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
        black_color = 0
        depth_image_3d = np.dstack((img_depth,img_depth,img_depth)) #depth image is 1 channel, color is 3 channels

        clipping_distance_in_centimeters = trackbar_depth.get_param_as_tuple()

        min_clipping_distance_in_meters = float(clipping_distance_in_centimeters[0][0]) / 100.0  #1 meter
        max_clipping_distance_in_meters = float(clipping_distance_in_centimeters[1][0]) / 100.0  #1 meter

        min_clipping_distance = min_clipping_distance_in_meters / depth_scale
        max_clipping_distance = max_clipping_distance_in_meters / depth_scale

        bg_removed_max = np.where((depth_image_3d > max_clipping_distance) | (depth_image_3d <= 0), black_color, img_bgr)
        bg_removed = np.where((depth_image_3d < min_clipping_distance) | (depth_image_3d <= 0), black_color, bg_removed_max)

        #apply filter to extract the red color
        Lmask_MIN, Lmask_MAX = trackbar_mask1.get_param_as_tuple()
        Umask_MIN, Umask_MAX = trackbar_mask2.get_param_as_tuple()
        # ipdb.set_trace()
        ###########################################################################################
        # gray_croped,croped = applyFilter(bg_removed,Lmask_MIN,Lmask_MAX,Umask_MIN,Umask_MAX)
        # print(gray_croped.shape)
        # box_index = np.where(croped != 0)

        ##cut the image to the range og left side tomato
        # if (len(box_index[0]) >= 10  and len(box_index[1]) >= 10): # check red exists
        #     #this is the tomato which is in left side
        #     target_left = np.amin(box_index[1])
        #     right = target_left + color_w/20

        if(True):            
            left_tomato_cropped_img = bg_removed[:,0:color_w/2]

            cv2.imshow('left_tomato_cropped_img', left_tomato_cropped_img)

            #get the result val and image obj
            red_area, extracted_image = get_colored_area(left_tomato_cropped_img,Lmask_MIN,Lmask_MAX,Umask_MIN,Umask_MAX)
    #################################################################

            #convert red-area to gray scale
            gray_extracted_image = cv2.cvtColor(extracted_image, cv2.COLOR_RGB2GRAY)
            #convert red-area-gray scale to binary img
            ret, threshold_img = cv2.threshold(gray_extracted_image, 1, 255, cv2.THRESH_BINARY)
            
            if(ret):
                #find contours
                image, contours, hierarchy = cv2.findContours(threshold_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                # get size of contour
                contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
                if(len(contours)>0):
                    #select biggest one
                    biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
                    cv2.drawContours(extracted_image, contours, -1, (255, 0, 0), 2)
                    (x,y),radius = cv2.minEnclosingCircle(biggest_contour)
                    #Get the total pixel of biggest_contour
                    red_area = cv2.contourArea(biggest_contour)
                    center = (int(x),int(y))
                    radius = int(radius)
                    num_of_cnts = len(contours)

                    #display window
                    if (enable_image_show):
                        cv2.putText(extracted_image, 'red pixel(total): '+str(red_area), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
                        cv2.putText(extracted_image, 'radius: ' + str(radius), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
                        cv2.putText(extracted_image, 'num of cnts: ' + str(num_of_cnts), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
                        cv2.circle(extracted_image, (int(x),int(y)),10,(0,0,255),-1)
                        cv2.circle(extracted_image,center,radius,(0,255,0),2)

                        ##show the image of handeye
                        cv2.namedWindow('main_cam_extracted', cv2.WINDOW_AUTOSIZE)
                        cv2.imshow('main_cam_extracted', cv2.cvtColor(extracted_image, cv2.COLOR_BGR2RGB))
                        cv2.imshow('binary', threshold_img)
                    # cv2.waitKey(1)

                    #deproject the depth on pixel coordinate
                    depth_pixel = [int(x), int(y)]
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, img_depth[depth_pixel[1],depth_pixel[0]]*depth_scale)

                    #target mark
                    hand_mark.counter = 0
                    t = rospy.get_time()
                    hand_mark.color = [1,0,0,1]
                    m = hand_mark.marker(points= [(depth_point[0], depth_point[1], depth_point[2])])
                    pub.publish(m)

                    cv2.circle(bg_removed, (int(x), int(y)), radius=7, color=(255,0,0), thickness=-1)

            else:
                #target mark
                hand_mark.counter = 0
                t = rospy.get_time()
                hand_mark.color = [1,0,0,1]
                m = hand_mark.marker(points= [(-10, -10, -10)])
                pub.publish(m)
                print('published dammy target')  

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
            cv2.imshow("bg_removed_BGR", bg_removed)
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
