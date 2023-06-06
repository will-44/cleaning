#!/usr/bin/env python3

# Import necessary packages
import cv2
import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image, CameraInfo
from threading import Lock

# Initialize publishers and variables
pcl_pub = rospy.Publisher('/depth_filter/image_raw', Image, queue_size=10)  # Publisher to publish modified depth map
camera_info_pub = rospy.Publisher('/depth_filter/camera_info', CameraInfo,
                                  queue_size=10)  # Publisher to publish camera info
bridge = CvBridge()  # Object to convert OpenCV image to ROS image
radius = 50  # Radius of dust spot
dust_poses = []  # Array to store dust pixel positions
camera_info = CameraInfo()  # Object to store camera info
path_mask = rospkg.RosPack().get_path('cleaning') + "/img/mask_depth.png"  # Path to the mask image
mask_tool = cv2.imread(path_mask)  # Load the mask image
mask_tool = cv2.cvtColor(mask_tool, cv2.COLOR_BGR2GRAY)  # Convert the mask image to grayscale

my_object_lock = Lock()  # Lock object to synchronize access to shared variable


def callback_dust(msg):
    """
    Callback function to receive the dust pixel positions
    :param msg: Floats array 1D
    :return:
    """
    my_object_lock.acquire()  # Acquire the lock to synchronize access to shared variable
    dust_poses = np.reshape(msg.data, (int(len(msg.data) / 2), 2))  # Convert the dust pixel positions to numpy array
    my_object_lock.release()  # Release the lock to allow other threads to access the shared variable


def callback_depth(img):
    """
    Callback function to receive the depth map from camera
    :param img: 2D image
    :return:
    """
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "16UC1")  # Convert ROS image to OpenCV image in millimeters
    except CvBridgeError as e:
        # print(e)
        return

    try:
        # Remove the tool from depth map
        cv_image[mask_tool != 0] = 0
        # Remove the dust spots
        mask_dust = np.zeros_like(cv_image)

        my_object_lock.acquire()  # Acquire the lock to synchronize access to shared variable
        for dust in dust_poses:
            cv2.circle(mask_dust, dust.astype(int), radius, 1, thickness=-1)  # Create a mask for each dust spot
        my_object_lock.release()  # Release the lock to allow other threads to access the shared variable

        cv_image[mask_dust == 1] = cv_image[
                                       mask_dust == 1] + 50  # Increase the depth values of pixels inside dust spot by 50mm

        # Convert modified depth map to ROS image and publish
        image_msg = bridge.cv2_to_imgmsg(cv_image, "16UC1")  # Convert the modified depth map to a ROS image message

        image_msg.header.frame_id = "rgb_camera_link"  # Set the frame ID of the image message

        now = rospy.Time.now()
        image_msg.header.stamp = now
        pcl_pub.publish(image_msg)  # Publish the modified depth map

        camera_info.header.stamp = now
        camera_info_pub.publish(camera_info)  # Publish the camera info

    except CvBridgeError as e:
        print("erroe")
        return


if __name__ == '__main__':
    rospy.init_node('depth_limit', anonymous=True)  # Initialize the ROS node

    dust_sub = rospy.Subscriber("/dust_pixels", Floats, callback_dust)  # Subscribe to dust pixel positions
    depth_sub = rospy.Subscriber("/depth_to_rgb/image_raw", Image, callback_depth)  # Subscribe to depth map

    camera_info = rospy.wait_for_message("/depth_to_rgb/camera_info", CameraInfo)  # Wait for camera info message
    rospy.loginfo("Depth limit ready")  # Log a message indicating readiness
    rospy.spin()  # Start the ROS node, continue spinning and processing incoming messages
