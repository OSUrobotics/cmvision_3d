#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from image_geometry import PinholeCameraModel
from tf2_msgs.msg import TFMessage, TransformStamped
from cmvision.msg import Blobs, Blob

import cv, cv2
import numpy as np

#The "model" part of our model-view-controller. 
#Each color_model represents a color that we're tracking. When calling update(), you present new information.
class color_model():
    def __init__(self, blob, parent_frame, depth_image, cam_model):
        self.blob = blob
        self.parent_frame = parent_frame
        self.depth_image = depth_image
        self.cam_model = cam_model
    #Updates the model with more information. Returns false if this information is rejected and true if this information is accepted.
    def update(blob, depth_image, cam_model):
        return True

    #Publishes to our view, color_broadcaster, if the model updates. 
    def publish():
    	return self._toTransform()
## Private functions
## ^^^^^^^^^^^^^^^^^
	
	#Takes our data and makes a tf2 transform message.
	def _toTransform():
		transform = TransformStamped()
		transform.header.stamp = rospy.Time.now()
		transform.header.frame_id = self.parent_frame
		transform.child_frame_id = self.blob.name

		(x,y,z) = self._projectTo3d(self.blob.x, self.blob.y)
		transform.transform.translation.x = x
		transform.transform.translation.y = y
		transform.transform.translation.z = z

		transform.transform.rotation.w = 1.0

	def _projectTo3d(x, y):
		[vx,vy,vz] = self.cam_model.projectPixelTo3dRay((x,y))
		blob_z = self._getDepthAt(x,y)
		blob_x = vx * blob_z
		blob_y = vy * blob_z

		return (blob_x, blob_y, blob_z)

	def _getDepthAt(x,y):
		return self.depth_image[y][x]

