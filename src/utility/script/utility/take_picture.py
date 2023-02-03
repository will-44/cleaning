#!/usr/bin/env python3

import sys
import rospy
import cv2
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np
import time 
import argparse
import imutils
import rospkg
from getkey import getkey, keys

"""
This class connect to the image topics from the kinect and save the actual image 
"""
class takePictures:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback)
        self.img = rospy.wait_for_message("/rgb/image_raw", Image, timeout=10)
        try:
            self.img = self.bridge.imgmsg_to_cv2(self.img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Image error")

    # This methode get the incomming image from the camera 
    def callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Image error")

if __name__ == '__main__':
    rospy.init_node('take_picture', anonymous=True)
    pict = takePictures()
    print("Press q to take a picture")

    while not rospy.is_shutdown():
        if(getkey() == 'q'):
            path = rospkg.RosPack().get_path('utility') + f"/img/image_{time.time()}.jpg"
            cv2.imwrite(path, pict.img)
            print("Image save")
        rospy.sleep(0.1)
        