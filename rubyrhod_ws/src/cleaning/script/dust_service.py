#!/usr/bin/env python3

import open3d as o3d  # Importing the open3d library for point cloud processing
import rospy  # Importing the rospy package for ROS (Robot Operating System) integration
from open3d_ros_helper import open3d_ros_helper as o3d_ros  # Importing the open3d_ros_helper package

from cleaning.srv import DetectDust, DetectDustResponse  # Importing the ROS service message types
from detect_dust import detectDust  # Importing the detectDust class for dust detection


def detect_dust_srv(req):
    """
    Service callback function for detecting dust from 2D and 3D image.
    :param req: Service request containing depth and color images
    :return: Service response containing the detected dust point cloud
    """
    # Convert the color and depth images from ROS format to OpenCV format
    image_color = detector.bridge.imgmsg_to_cv2(req.color_img, "bgr8")
    image_depth = detector.bridge.imgmsg_to_cv2(req.depth_img, "16UC1")

    # Detect dust based on the color image
    dust_center = detector.detect_dust(image_color)
    print("dust_center" + str(len(dust_center)))
    print(dust_center)
    # Get the positions of dust centers in 3D space using the depth image
    poses = detector.get_position(dust_center, image_depth)
    print("poses" + str(len(poses)))
    print(poses)
    # Convert the dust positions to an open3d PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(poses)
    pcd.paint_uniform_color([0, 0, 1])

    # Convert the open3d PointCloud object to a ROS PointCloud2 message
    msg_pcd = o3d_ros.o3dpc_to_rospc(pcd, frame_id="rgb_camera_link")

    # Create the service response containing the detected dust point cloud
    answer = DetectDustResponse()
    answer.pcds = [msg_pcd]
    return answer


if __name__ == "__main__":
    # Initialize the ROS node with the name 'detect_dust_server'
    rospy.init_node('detect_dust_server')

    # Create a ROS service named 'detect_dust' with the service callback function 'detect_dust_srv'
    s = rospy.Service('detect_dust', DetectDust, detect_dust_srv)

    # Create an instance of the detectDust class with 'callback_activate' set to False
    detector = detectDust(callback_activate=False)

    # Print a message indicating that the system is ready to detect dust
    print("Ready to detect dust")

    # Spin the ROS node, continuously processing incoming requests
    rospy.spin()
