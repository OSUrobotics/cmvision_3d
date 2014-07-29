#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo 
from cmvision.msg import Blobs

from cv_bridge import CvBridge

import cv, cv2
import numpy as np
import copy
from color_model import color_model

#This package integrates cmvision with tf and localization; now we can track color in 3D.
class color_controller():
    def __init__(self, depth_topic, info_topic, map_topic):
        #We are using a depth image to get depth information what we're tracking.
        rospy.Subscriber(depth_topic, Image, self.depth_callback)

        #We need CameraInfo in order to use PinholeCameraModel below.
        rospy.Subscriber(info_topic, CameraInfo, self.camera_callback)
        self.hasCameraInfo = False

        #This package is just an extension of cmvision to provide tf tracking of the blobs provided by cmvision. 
        rospy.Subscriber('blobs', Blobs, self.blob_callback)

        #To take our Ros Image into a cv message and subsequently a numpy array.
        self.bridge = CvBridge()        

        # To make the pixel to vector projection
        self.camModel = PinholeCameraModel()

        #We are integrating cmvision with localization and thus want to know which frame we should put our colors into.
        self.map_topic = map_topic

        #array of our color_models.
        self.colors = []

    def blob_callback(self, blobs):
        for blob in blobs.blobs:
            pass


    def depth_callback(self, image):
        image_cv = self.bridge.imgmsg_to_cv(image, '32FC1')
        image_cv2 = np.array(image_cv, dtype=np.float32)
        self.depth_image = image_cv2


    def camera_callback(self, info):
        if not self.hasCameraInfo:
            self.camModel.fromCameraInfo(cameraInfo)
        self.hasCameraInfo = True


if __name__ == '__main__':
    rospy.init_node('color_controller')
    boundingBox = color_controller('camera/depth/image', '/camera/rgb/camera_info', '/map')

    rospy.spin()
