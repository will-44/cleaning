#!/usr/bin/env python3

import rospy
from PIL.ImageChops import offset
from ros_numpy import numpify
import rospkg
from utility.doosan import Doosan
from utility.base_poses import PosesBase
from utility.robotiq import Robotiq
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose2D, Pose
from visualization_msgs.msg import Marker
from utility.rviz_tool import display_marker
from open3d_tools import Open3dTool
import tf2_ros
import tf
import tf2_geometry_msgs
import geometry_msgs.msg
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
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2
from open3d_ros_helper import open3d_ros_helper as o3d_ros

import fast_tsp
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.distances import great_circle_distance_matrix, euclidean_distance_matrix
import time
from open3d_tools import Open3dTool


class Robot:

    def __init__(self, debug=True):
        self.debug = debug
        self.open3d_tool = Open3dTool()

    def visualize_pcds(self, *args):
        """

        :param args:
        :return:
        """
        if self.debug:
            pcds = []
            for arg in args:
                if isinstance(arg, list):
                    pcds.extend(arg)
                elif isinstance(arg, np.array()):
                    pcds.append(self.open3d_tool.np2pcd(arg))
                elif isinstance(arg, o3d.geometry.PointCloud()):
                    pcds.append(arg)
                elif isinstance(arg, o3d.geometry.LineSet()):
                    pcds.append(arg)
                else:
                    rospy.loginfo("VISUALIZER: arg type not accepted")

            o3d.visualization.draw_geometries(pcds)
