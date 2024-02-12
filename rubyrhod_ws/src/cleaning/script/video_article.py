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
from tf_tools import TfTools

class VideoArticle:
    """
    This class get the dust point and the dust photo to show points accepeted and rejected
    An other function save the rosbag with only point cloud and rviz markers
    """

    def __init__(self):
        self.tf_tools = TfTools()
        self.bridge = CvBridge()

        self.camera_info = rospy.wait_for_message("/rgb/camera_info", CameraInfo, timeout=10)
        try:
            self.intrinsec = np.reshape(self.camera_info.K, (3, 3))
        except:
            rospy.loginfo('{nodeName} : No camera info'.format(nodeName=rospy.get_name()))

    def convert_3d_to_2d(self, points):
        """
        This methode convert 3d points to 2d pixel with intrinsic camera
        :param points: posestamped array
        :return:
        """
        points_2d = []
        for point in points:
            pix_x = point.pose.position.x * self.intrinsec[0, 0] / point.pose.position.z + self.intrinsec[0, 2]
            pix_y = point.pose.position.y * self.intrinsec[1, 1] / point.pose.position.z + self.intrinsec[1, 2]
            points_2d.append([pix_x, pix_y])
        return points_2d

    def points_to_picture(self, points_accepted, point_rejected, image):
        """
        This methode get the dust point accepted and rejected and show them in the 2d_picture.
        This picture is send in the topic /image_video
        :param points_accepted: 3d array in the vacuum_tcp frame
        :param point_rejected:  3d array in the vacuum_tcp frame
        :param image:
        :return:
        """
        # transform the picture from ros to opencv
        try:

            image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            cv2.imwrite(rospkg.RosPack().get_path('cleaning') + "/img/image_dust" +
                        time.strftime("%Y%m%d-%H%M%S") + ".png", image)

            # hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        except CvBridgeError as e:
            print(e)
        # transform points from tcp frame to camera frame and convert point dust to 2d pixel with intrasec camera
        trans = self.tf_tools.get_transform("vacuum_tcp", "rgb_camera_link")
        pixel_rejected_rgb = self.convert_3d_to_2d(self.tf_tools.transform_pose_array(self.tf_tools.array_2_posestamped(point_rejected), trans))
        pixel_accepted_rgb = self.convert_3d_to_2d(self.tf_tools.transform_pose_array(self.tf_tools.array_2_posestamped(points_accepted), trans))

        # show accepted points in green and rejected in red on the image
        for point in pixel_accepted_rgb:
            cv2.circle(image, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)
        for point in pixel_rejected_rgb:
            cv2.circle(image, (int(point[0]), int(point[1])), 5, (0, 0, 255), -1)

        #save image in the file image of the ros package cleaning with ros time
        cv2.imwrite(rospkg.RosPack().get_path('cleaning') + "/img/image_dust_valid" +
                    time.strftime("%Y%m%d-%H%M%S") + ".png", image)

