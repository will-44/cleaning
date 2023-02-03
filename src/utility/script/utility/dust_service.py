#!/usr/bin/env python3

import rospy
from doosan import Doosan
from robotiq import Robotiq
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker
import rviz_tool
import tf
import open3d as o3d
import numpy as np
from open3d_ros_helper import open3d_ros_helper as o3d_ros

from sensor_msgs.msg import PointCloud2

from utility.srv import DetectDust, DetectDustResponse

from detect_dust import detectDust



def detect_dust(req):
    global detector
    result_pub = rospy.Publisher('/dust', PointCloud2, queue_size=10)

    image_color =  detector.bridge.imgmsg_to_cv2(req.color_img, "bgr8")
    image_depth =  detector.bridge.imgmsg_to_cv2(req.depth_img, "16UC1")

    dust_center = detector.detect_dust(image_color)
    poses = detector.get_position(dust_center, image_depth)

    print(poses)
    # Convert array to open3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(poses)
    pcd.paint_uniform_color([0, 0, 1])
    # R = pcd.get_rotation_matrix_from_xyz((0, 0, -np.pi/2))
    # pcd.rotate(R, center=(0, 0, 0))
    # o3d.visualization.draw_geometries([pcd])
    msg_pcd = o3d_ros.o3dpc_to_rospc(pcd, frame_id="rgb_camera_link")

    result_pub.publish(msg_pcd)

    answer = DetectDustResponse()
    answer.pcds = [msg_pcd]
    return answer

if __name__ == "__main__":
    global detector
    rospy.init_node('detect_dust_server')
    s = rospy.Service('detect_dust', DetectDust, detect_dust)

    detector = detectDust(callback_activate = False)

    print("Ready to detect dust")
    rospy.spin()