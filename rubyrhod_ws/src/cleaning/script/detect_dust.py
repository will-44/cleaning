#!/usr/bin/env python3

import sys
import time

import rospkg
import rospy
import cv2
from std_msgs.msg import String, Float64MultiArray, Int32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np
from rospy_tutorials.msg import Floats

import argparse
import imutils


class detectDust:

    def __init__(self, callback_activate=True):
        self.callback_activate = callback_activate
        # self.image_pub = rospy.Publisher("image_topic_2", Float64MultiArray, queue_size=10)
        self.image_pub = rospy.Publisher("/image_topic_2", Image, queue_size=1)
        self.depth_pub = rospy.Publisher("/image_topic_depth", Image, queue_size=1)
        self.dust_pixels_pub = rospy.Publisher("/dust_pixels", Floats, queue_size=1)
        self.bridge = CvBridge()

        self.depth = 0
        self.camera_info = rospy.wait_for_message("/rgb/camera_info", CameraInfo, timeout=10)
        path_mask = rospkg.RosPack().get_path('cleaning') + "/img/mask_depth.png"  # Path to the mask image
        path_mask_depth = rospkg.RosPack().get_path('cleaning') + "/img/mask_depth.png"  # Path to the mask image

        mask_tool_depth = cv2.imread(path_mask)  # Load the mask image
        mask_tool = cv2.imread(path_mask)  # Load the mask image

        self.mask_tool = cv2.cvtColor(mask_tool, cv2.COLOR_BGR2GRAY)  # Convert the mask image to grayscale
        self.mask_tool_depth = cv2.cvtColor(mask_tool_depth, cv2.COLOR_BGR2GRAY)  # Convert the mask image to grayscale

        try:
            self.intrinsec = np.reshape(self.camera_info.K, (3, 3))
            print(self.intrinsec)
        except:
            rospy.loginfo('{nodeName} : No camera info'.format(nodeName=rospy.get_name()))

        # lo_color = (34, 106, 56)
        # hi_color = (92, 255, 133)
        self.min_hue = 34
        self.max_hue = 92
        self.min_saturation = 106
        self.max_saturation = 255
        self.min_value = 56
        self.max_value = 133

        if callback_activate:
            self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback)

            self.max_hue_sub = rospy.Subscriber("/max_hue", Int32, self.callback_max_hue)
            self.max_value_sub = rospy.Subscriber("/max_value", Int32, self.callback_max_value)
            self.max_saturation_sub = rospy.Subscriber("/max_saturation", Int32, self.callback_max_saturation)

            self.min_hue_sub = rospy.Subscriber("/min_hue", Int32, self.callback_mim_hue)
            self.min_value_sub = rospy.Subscriber("/min_value", Int32, self.callback_mim_value)
            self.min_saturation_sub = rospy.Subscriber("/min_saturation", Int32, self.callback_mim_saturation)

            self.depth_sub = rospy.Subscriber("/depth_to_rgb/image_raw", Image, self.callback_depth)

    def callback_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def callback_max_hue(self, msg):
        self.max_hue = msg.data

    def callback_max_saturation(self, msg):
        self.max_saturation = msg.data

    def callback_max_value(self, msg):
        self.max_value = msg.data

    def callback_mim_hue(self, msg):
        self.min_hue = msg.data

    def callback_mim_saturation(self, msg):
        self.min_saturation = msg.data

    def callback_mim_value(self, msg):
        self.min_value = msg.data

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv2.imwrite('Test_gray.jpg', cv_image)
        except CvBridgeError as e:
            print(e)
        center = self.detect_dust(cv_image)
        centers_poses = self.get_position(center, self.depth)
        # print(centers_poses)

    def get_position(self, center, depth):
        # remove from mask
        depth[self.mask_tool_depth != 0] = 0
        try:
            image_msg = self.bridge.cv2_to_imgmsg(depth, "16UC1")
            image_msg.header.frame_id = "rgb_camera_link"

            self.depth_pub.publish(image_msg)
        except CvBridgeError as e:
            print(e)

        # Get pose from 3d image
        center_poses = []
        cx = self.intrinsec[0][2]
        cy = self.intrinsec[1][2]
        fx = self.intrinsec[0][0]
        fy = self.intrinsec[1][1]
        for pixel in center:
            z = depth[pixel[1], pixel[0]]  # in meter
            x = ((pixel[0] - cx) * z / fx) / 1000
            y = ((pixel[1] - cy) * z / fy) / 1000
            z = z / 1000
            center_poses.append([x, y, z])

        return center_poses

    def detect_dust(self, image):

        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Remove dead zone cause of the tool
        hsv_image[self.mask_tool != 0] = [0, 0, 0]

        lo_color = (self.min_hue, self.min_saturation, self.min_value)
        hi_color = (self.max_hue, self.max_saturation, self.max_value)
        # print(lo_color)
        # For Canplex
        # lo_color = (36, 158, 28)#(34, 106, 56)
        # hi_color = (92, 255, 133)
        # For Sycodal
        # lo_color = (25, 106, 56)
        # hi_color = (92, 255, 133)
        #
        # For ETS
        lo_color = (30, 72, 24)
        hi_color = (81, 255, 255)
        mask = cv2.inRange(hsv_image, lo_color, hi_color)

        kernel_dilate = np.ones((10, 10), 'uint8')
        kernel_erode = np.ones((5, 5), 'uint8')

        erode_mask = cv2.erode(mask, kernel_erode)
        dilate_mask = cv2.dilate(erode_mask, kernel_dilate, iterations=1)
        result = cv2.bitwise_and(image, image, mask=dilate_mask)
        result = dilate_mask

        # find contours in the thresholded image
        cnts = cv2.findContours(dilate_mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        centers = []
        # loop over the contours
        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            if (M["m00"] != 0):  # don't know why could it be 0
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centers.append([cX, cY])

                # draw the contour and center of the shape on the image
                cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                cv2.circle(image, (cX, cY), 3, (0, 0, 255), -1)

        try:
            # if not self.callback_activate:
            #     cv2.imshow('image', image)
            #     cv2.waitKey(0)
            image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            image_msg.header.frame_id = "rgb_camera_link"

            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            print(e)
        # msg = Floats()
        # msg.data = np.reshape(centers, (np.size(centers),))
        # self.dust_pixels_pub.publish(msg)

        if not self.callback_activate:
            print("nb of dust: " + str(len(centers)))
            print(centers)
            cv2.imwrite(rospkg.RosPack().get_path('cleaning') + "/img/original_image" +
                        time.strftime("%Y%m%d-%H%M%S") + ".png", image)
        return centers


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = detectDust()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
