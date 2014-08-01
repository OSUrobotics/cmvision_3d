#!/usr/bin/env python

import rospy
import tf

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

		#Subscribe to image for debugging.
		# rospy.Subscriber('camera/rgb/image_color', Image, self.image_callback)
		self.listener = tf.TransformListener()


		#Send to our view.
		self.view_pub = rospy.Publisher('color_tracker/tf', TFMessage)
		# self.view_pub = rospy.Publisher('tf_static', TFMessage)

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
					wasUpdated = wasUpdated or self.colors[blob.name].update(blob, self.depth_image)
				else:
					self.colors[blob.name] = color_model(blob, self.camera_info, self.parent_frame, self.depth_image, self.cam_model)
					wasUpdated = True

				#Publish it whether it's been updated or not.
				self.transforms.transforms.append( self.colors[blob.name].publish() )

		except():
			pass
		if wasUpdated:
			self.view_pub.publish(self.transforms)

	def depth_callback(self, image):
		image_cv = self.bridge.imgmsg_to_cv(image, '32FC1')
		image_cv2 = np.array(image_cv, dtype=np.float32)
		self.depth_image = image_cv2


	def camera_callback(self, camera_info):
		if not self.hasCameraInfo:
			self.cam_model.fromCameraInfo(camera_info)
			self.camera_info = camera_info
		self.hasCameraInfo = True
		
	def image_callback(self, image):
		image_cv = self.bridge.imgmsg_to_cv(image, 'bgr8')
		image_cv2 = np.array(image_cv, dtype=np.uint8)

		# self.listener.waitForTransform('/camera_rgb_optical_frame', '/green', rospy.Time.now(), rospy.Duration(0.5))
		if self.listener.frameExists('green'):
			(trans,rot) = self.listener.lookupTransform( '/camera_rgb_optical_frame', '/green',  rospy.Time(0))
			projected = self.cam_model.project3dToPixel(trans)
			point = (int(projected[0]), int(projected[1]))
			point2 = (int(projected[0])+50, int(projected[1]) + 50)
			cv2.rectangle(image_cv2, point, point2, -1, -1)
		cv2.imshow('thing', image_cv2)
		cv2.waitKey(3)


if __name__ == '__main__':
	rospy.init_node('color_controller')
	depth_image = rospy.get_param("color_controller/depth_image", "camera/depth_registered/image_raw")
	camera_topic = rospy.get_param("color_controller/camera_topic", '/camera/rgb/camera_info')
	parent_frame = rospy.get_param("color_controller/parent_frame", '/camera_rgb_optical_frame')
	my_controller = color_controller(depth_image, camera_topic, parent_frame)

	rospy.spin()
