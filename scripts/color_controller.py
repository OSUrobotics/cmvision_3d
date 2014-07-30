#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image, CameraInfo 
from tf2_msgs.msg import TFMessage
from image_geometry import PinholeCameraModel
from cmvision.msg import Blobs

from cv_bridge import CvBridge

import cv, cv2
import numpy as np
import copy
from color_model import color_model

#This package integrates cmvision with tf and localization; now we can track color in 3D.
class color_controller():
    def __init__(self, depth_topic, info_topic, parent_frame):
        #We are using a depth image to get depth information of what we're tracking.
        rospy.Subscriber(depth_topic, Image, self.depth_callback)

        #We need CameraInfo in order to use PinholeCameraModel below.
        rospy.Subscriber(info_topic, CameraInfo, self.camera_callback)
        self.hasCameraInfo = False

        #This package is just an extension of cmvision to provide tf tracking of the blobs provided by cmvision. 
        rospy.Subscriber('blobs', Blobs, self.blob_callback)

        #Send to our view.
        self.view_pub = rospy.Publisher('color_tracker/tf', TFMessage)

        #To take our Ros Image into a cv message and subsequently a numpy array.
        self.bridge = CvBridge()        

        # To make the pixel to vector projection
        self.cam_model = PinholeCameraModel()

        #We are integrating cmvision with localization and thus want to know which frame we should put our colors into.
        self.parent_frame = parent_frame

        #array of our color_models.
        self.colors = {}

        #Keep track of our transforms to send them to color_broadcaster.
        self.transforms = TFMessage()

        #blobs is received from running cmvision. It's color blobs as defined by our color file.
    def blob_callback(self, blobs):
        self.transforms = TFMessage()
        wasUpdated = False
        try:
            for blob in blobs.blobs:
                #If this blob already exists, update our idea of it. Otherwise create another color_model.
                if blob.name in self.colors:
                    if (self.colors[blob.name].update(blob, self.depth_image, self.cam_model)):
                        wasUpdated = True
                else:
                    self.colors[blob.name] = color_model(blob, self.parent_frame, self.depth_image, self.cam_model)
        
                #Publish it whether it's been updated or not.
                self.transforms.transforms.append(self.colors[blob.name].publish() )

        except():
            pass
        if wasUpdated:
            self.view_pub.publish(self.transforms)

    def depth_callback(self, image):
        image_cv = self.bridge.imgmsg_to_cv(image, '32FC1')
        image_cv2 = np.array(image_cv, dtype=np.float32)
        self.depth_image = image_cv2


    def camera_callback(self, cameraInfo):
        if not self.hasCameraInfo:
            self.cam_model.fromCameraInfo(cameraInfo)
        self.hasCameraInfo = True


if __name__ == '__main__':
    rospy.init_node('color_controller')
    depth_image = rospy.get_param("color_controller/depth_image", "camera/depth/image")
    camera_topic = rospy.get_param("color_controller/camera_topic", '/camera/rgb/camera_info')
    parent_frame = rospy.get_param("color_controller/parent_frame", '/camera_rgb_optical_frame')
    boundingBox = color_controller('camera/depth/image', '/camera/rgb/camera_info', '/camera_rgb_optical_frame')

    rospy.spin()
