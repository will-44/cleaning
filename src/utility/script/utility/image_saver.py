#!/usr/bin/env python3

from ast import arg
import sys
from pathlib import Path
from datetime import datetime
import traceback
from cv_bridge import CvBridge
import cv2
import argparse

# ROS imports
import rospy
from sensor_msgs.msg import CompressedImage

# Global variable holding the path to the folder where rosbags will be stored
_image_folder=str(Path.home()) + "/image/"
#_image_folder="/media/USBDrive" + "/image/"

class ImageSaver:
    def __init__(self, camera, duration):
        self.bridge = CvBridge()
        self.camera = camera

        rospy.Subscriber(f"/{camera}/color/image_raw/compressed", CompressedImage, self.new_image_callback)

        rospy.Timer(rospy.Duration(duration), self.save_image)

    def new_image_callback(self, data):
        self.image = data

    def save_image(self, _):
        global _image_folder

        image = self.image
        now  = datetime.now()

        cv_image = self.bridge.compressed_imgmsg_to_cv2(image)

        cv2.imwrite(_image_folder + self.camera + "_" + now.strftime("%Y_%m_%d_%H_%M_%S") + ".jpg", cv_image)

if __name__ == "__main__":
    parser  = argparse.ArgumentParser(description="saves image from [camera] to image file every [--duration, -d] seconds")
    parser.add_argument("camera", help="namespace associated with the camera of interest")
    parser.add_argument("-d", "--duration", help="period in second between image recording", type=int, default=1)
    argv = rospy.myargv(sys.argv)
    args=parser.parse_args(argv[1:])

    try:
        rospy.init_node(f"image_saver_{args.camera}", anonymous=False)
        
        ImageSaver(args.camera, args.duration)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())