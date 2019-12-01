#!/usr/bin/env python  
import roslib
import rospy
import tf
import numpy as np
import sys
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped,Pose,Point

class eef_jog():
    def __init__(self,LorR):
        self._LorR = LorR
        self._target_frame = "/"+ LorR +"EEFlink"  ###FROM
        self._reference_frame = "/"+ LorR +"link0" ##TO
        self._listener = tf.TransformListener()
    
    def create_point(self,point):
        msg = Point()
        msg.x = point[0]
        msg.y = point[1]
        msg.z = point[2]
        return msg

    ####rospy.Duration ... this param can be changed
    def transform(self,target):
        self._listener.waitForTransform(self._reference_frame, self._target_frame, rospy.Time(0),rospy.Duration(4.0))
        try:
            #transform position from target_frame to reference frame
            target_msg = self.create_point(target)
            print(target_msg)
            target_ref_eef=PointStamped()
            target_ref_eef.header.frame_id = self._target_frame
            target_ref_eef.header.stamp =rospy.Time(0)
            target_ref_eef.point = target_msg
            p=self._listener.transformPoint(self._reference_frame,target_ref_eef)
            points= (p.point.x, p.point.y, p.point.z)
            print('----------------------target_ref_' + self._LorR + 'EEF is calculated')
            return points
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return (0,0,0)

if __name__ == '__main__':
    rospy.init_node('eef_tf_listener',anonymous=True)
    print('start eef_tf_listener')
    eef_jog = eef_jog('L')
    result = eef_jog.transform([0,0,0.1])
    print(result)

