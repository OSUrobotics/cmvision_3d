#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Image, CameraInfo 
from cmvision.msg import Blobs
from tf2_msgs.msg import TFMessage

from cv_bridge import CvBridge

import cv, cv2
import numpy as np
import copy

class color_broadcaster():
    def __init__(self, topic):
        self.sub = rospy.Subscriber(topic, TFMessage, self.tf_callback)
        self.broadcaster = tf.TransformBroadcaster()
        self.transforms = TFMessage()

    def tf_callback(self, transforms): 
        self.transforms = transforms

if __name__ == '__main__':
    rospy.init_node('color_broadcaster')
    cb = color_broadcaster('color_tracker/tf')
    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        for transform in cb.transforms.transforms:
            pos = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
            rot = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)
            cb.broadcaster.sendTransform(pos, rot, rospy.Time.now(), transform.child_frame_id, transform.header.frame_id)

    rospy.spin()
