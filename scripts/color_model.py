#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import tf

from image_geometry import PinholeCameraModel
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_msgs.msg import TFMessage
from cmvision.msg import Blobs, Blob

import cv, cv2
import numpy as np

#The "model" part of our model-view-controller. 
#Each color_model represents a color that we're tracking. When calling update(), you present new information.
class color_model():
	def __init__(self, blob, camera_info, parent_frame, depth_image, cam_model, listener, broadcaster):
		
		#Our initial blob information.
		self.blob = blob

		#The frame we wish our blobs to have as a parent. E.g, this is "/map" if I'm localizing to the /map frame.
		self.parent_frame = parent_frame
		
		#Depth image is important for projecting the blob to 3D.
		self.depth_image = depth_image
		
		#Our projected color blob exists in this frame.
		self.camera_frame = camera_info.header.frame_id
		
		#Cam_model is important for projections to and from 2d/3d.
		self.cam_model = cam_model

		#Listener is necessary to transform from camera_frame to parent_frame.
		self.listener = listener

		#Broadcaster to publish the transforms.
		self.broadcaster = broadcaster

	#Updates the model with more information. Returns false if this information is rejected and true if this information is accepted.
	def update(self, blob, depth_image):
		self.blob = blob
		self.depth_image = depth_image
		return True

	#Publishes to our view, color_broadcaster, if the model updates. 
	def publish(self):
		transform = self._toTransform()
		pos = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
		rot = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)

		self.broadcaster.sendTransform(pos, rot, rospy.Time.now(), transform.child_frame_id, transform.header.frame_id)

		return transform

## Private functions
## ^^^^^^^^^^^^^^^^^
	
	#Takes our data and makes a tf2 transform message.
	def _toTransform(self):
		transform = TransformStamped()
		transform.header.stamp = rospy.Time.now()
		transform.header.frame_id = self.camera_frame
		transform.child_frame_id = self.blob.name

		(x,y,z) = self._projectTo3d(self.blob.x, self.blob.y)
		transform.transform.translation.x = x
		transform.transform.translation.y = y
		transform.transform.translation.z = z

		transform.transform.rotation.w = 1.0

		#If our parent frame is not the camera frame then an additional transformation is required.
		if self.parent_frame != self.camera_frame:
			point = PointStamped()
			point.header.frame_id = self.camera_frame
			point.header.stamp = rospy.Time(0)
			point.point.x = transform.transform.translation.x
			point.point.y = transform.transform.translation.y
			point.point.z = transform.transform.translation.z

			#Now we've gone from the regular camera frame to the correct parent_frame.
			point_transformed = self.listener.transformPoint(self.parent_frame, point)
			
			transform.header.frame_id = self.parent_frame
			transform.transform.translation.x = point_transformed.point.x
			transform.transform.translation.y = point_transformed.point.y
			transform.transform.translation.z = point_transformed.point.z

		return transform

	def _projectTo3d(self, x, y):
		[vx,vy,vz] = self.cam_model.projectPixelTo3dRay((x,y))
		blob_z = self._getDepthAt(x,y)
		blob_x = vx * blob_z
		blob_y = vy * blob_z

		return (blob_x, blob_y, blob_z)

	def _getDepthAt(self, x,y):
		return self.depth_image[y][x]/1000

