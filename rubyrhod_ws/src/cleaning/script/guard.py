#!/usr/bin/env python3

import rospy
from PIL.ImageChops import offset
from ros_numpy import numpify
import rospkg
from utility.doosan import Doosan
from utility.base_poses import PosesBase
from utility.robotiq import Robotiq

from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from utility.rviz_tool import display_marker

import tf2_ros
import tf
import tf2_geometry_msgs

import pickle
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from math import radians, degrees, atan2
import copy
import random as rnd
from alive_progress import alive_bar
import glob
from scipy.spatial.transform import Rotation as R

from tf.transformations import euler_from_quaternion, quaternion_from_euler, \
    quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, Pose2D
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2
from open3d_ros_helper import open3d_ros_helper as o3d_ros

import fast_tsp
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.distances import great_circle_distance_matrix, euclidean_distance_matrix
import time
from open3d_tools import Open3dTool
from data_manager import DataManager
from robot import Robot


if __name__ == '__main__':
    # rospy.init_node('guards', anonymous=True)
    o3d_tool = Open3dTool()
    data_manager = DataManager()
    file_name = f'/data/offline_trajectory_table.pkl'
    rospkg.RosPack().get_path('cleaning') + file_name
    guards = data_manager.load_var_pickle(rospkg.RosPack().get_path('cleaning') + file_name)
    nb_guard = 0
    print(guards.values())
    # for val in guards.values():
    #     nb_guard += len(val)
    # print(nb_guard)
