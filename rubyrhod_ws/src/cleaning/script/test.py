#!/usr/bin/env python3

import pickle

import geometry_msgs.msg
import numpy as np
import rospkg
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
from open3d_ros_helper import open3d_ros_helper as o3d_ros
from ros_numpy import numpify
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from utility.doosan import Doosan
from utility.rviz_tool import display_marker_array
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

from data_manager import DataManager
from open3d_tools import Open3dTool
from robot import Robot
from utility.srv import DetectDust
from python_tsp.distances import euclidean_distance_matrix
from dust import Dust


rospy.init_node('test', anonymous=True)

arm = Doosan()
# # ask the robot position while ros exist
# while not rospy.is_shutdown():
#     # arm.dsr_get_joint_pos()
#     rospy.loginfo(arm.get_pos_j())

# ask dust pose
dust = Dust()
rospy.sleep(1)
dust_poses = dust.ask_dust_poses()

print(dust_poses)
# arm.set_tcp("vacuum_safe_tcp")
arm.dsr_set_tcp("vacuum")
# start compliance
arm.set_compliance([100, 100, 100, 20, 20, 20])

tcp_pose = arm.dsr_get_pose()
dust_pose = [dust_poses[0][0], dust_poses[0][1], dust_poses[0][2] + 0.01,
             tcp_pose[3], tcp_pose[4], tcp_pose[5]]
print(dust_pose)
# send the robot to the dust pose
arm.dsr_go_to_l(dust_pose)
