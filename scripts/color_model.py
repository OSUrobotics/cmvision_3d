#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from cmvision.msg import Blobs

import cv, cv2
import numpy as np

#The "model" part of our model-view-controller. 
#Each color_model represents a color that we're tracking. When calling update(), you present new information.
class color_model():
    def __init__(self):
        pass

    #Updates the model with more information. Returns false if this information is rejected and true if this information is accepted.
    def update():
        pass

    #Publishes to our view, color_broadcaster, if the model updates. 
    def publish():
        pass