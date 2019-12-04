#!/usr/bin/env python  
import roslib
from easy_markers.generator import *
import rospy
import tf
import numpy as np
import sys
import rospy
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped,Pose

if __name__ == '__main__':

    rospy.init_node('target_ref_link0', anonymous=True)
    # print('LorR = ',LorR)

    LorR  = str(rospy.get_param('~LorR'))

    #set color => L:green, R:blue
    color = [0,1,0,1] if(LorR=='L') else [0,0,1,1]

    target_marker_node = "/target_marker"
    print("waiting for  --/target_maker-- message")

    #frame to transform
    target_frame    = "/camera_color_optical_frame"   ######FROM
    reference_frame = "/"+ LorR +"link0"                        ####TO

    #Marker Publisher Initialize
    marker_pub = rospy.Publisher("/target_marker_" + LorR + "link0_frame", Marker, queue_size=10)
    target_ref_link0 = MarkerGenerator()
    target_ref_link0.type = Marker.CUBE_LIST
    target_ref_link0.scale = [.02]*3
    target_ref_link0.frame_id = reference_frame

    listener = tf.TransformListener()
    listener.waitForTransform(reference_frame, target_frame, rospy.Time(0),rospy.Duration(4.0))

    while not rospy.is_shutdown():
        try:
            #receive the target msg which is in target_frame
            target_msg = rospy.wait_for_message(target_marker_node, Marker)
            target_point = target_msg.points

            #transform position from target_frame to reference frame
            target_ref_camera=PointStamped()
            target_ref_camera.header.frame_id = target_frame
            target_ref_camera.header.stamp =rospy.Time(0)
            target_ref_camera.point = target_point[0]
        
            p=listener.transformPoint(reference_frame,target_ref_camera)
           
            # print (target_point)
            # print (p.point)
            target_ref_link0.counter = 0
            target_ref_link0.color = color
            m = target_ref_link0.marker(points= [(p.point.x, p.point.y, p.point.z)])

            if(int(target_point[0].x) == -10):
                m = target_ref_link0.marker(points= [(-1, -1, -1)])

            marker_pub.publish(m)
            print('----------------------target_marker_' + LorR + ' is published')
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
