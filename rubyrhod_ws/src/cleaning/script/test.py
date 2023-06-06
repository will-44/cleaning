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


arm = Doosan()

# Set tool weight
arm.dsr_emergency_stop()
res = arm.dsr_set_tool("Tool_clean")
arm.dsr_resume()
rospy.sleep(4)
# print(res)
arm.set_compliance()
# rospy.sleep(4)
res = arm.dsr_go_relatif([0.0, 0.0, -65.0, 0.0, 0.0, 0.0])
res = arm.dsr_go_relatif([0.0, 0.0, -65.0, 0.0, 0.0, 0.0])
rospy.sleep(2)
res = arm.dsr_go_relatif([0.0, 0.0, 65.0, 0.0, 0.0, 0.0])
rospy.sleep(3)
arm.release_compliance()
