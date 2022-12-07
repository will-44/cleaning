#!/usr/bin/env python3

import sys
import rospy
import cv2
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np

import argparse
import imutils


class detectDust:

    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2", Float64MultiArray, queue_size=10)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.detect_dust(cv_image)
        # (rows, cols, channels) = cv_image.shape
        # if cols > 60 and rows > 60:
        #     cv2.circle(cv_image, (50, 50), 10, 255)
        #
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)



    # def pixel_to_3d_coord(self, image_x, image_y):


    def detect_dust(self, image):

        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        lo_color = (30, 64, 90)
        hi_color = (100, 230, 180)
        mask = cv2.inRange(hsv_image, lo_color, hi_color)
        result = cv2.bitwise_and(image, image, mask=mask)

        # find contours in the thresholded image
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
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
                # cv2.putText(image, "center", (cX - 20, cY - 20),
                # 	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # show the image
                # cv2.imshow("Image", image)
        try:
            image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            image_msg.header.frame_id = "camera_body"

            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            print(e)

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
