#!/usr/bin/env python3
import numpy as np
import rospy
from doosan import Doosan
from rviz_tool import display_marker_array
import open3d as o3d
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

if __name__ == '__main__':
    # open the doosan class to set the colissions
    rospy.init_node('add_basic_colision', anonymous=True)
    arm = Doosan()
    rm_path = "/home/will/Sycobot/rubyrhod_ws/src/utility/mesh/rm.pcd"
    rm_pcd = o3d.io.read_point_cloud(rm_path)
    rm_pcd_np = np.asarray(rm_pcd.points)

    poses_list = []
    for pose_np in rm_pcd_np:
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = pose_np[0]
        pose_msg.pose.position.y = pose_np[1]
        pose_msg.pose.position.z = pose_np[2]
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = 1
        poses_list.append(pose_msg)

    display_marker_array(marker_type=Marker.SPHERE, poses=poses_list, ref="base_0")
