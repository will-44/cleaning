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
    rospy.init_node('evaluation', anonymous=True)
    o3d_tool = Open3dTool()
    data_manager = DataManager()
    # Load machine pcd
    machine_path = rospkg.RosPack().get_path('cleaning') + "/mesh/scie_3.ply"
    machine_pcd = o3d.io.read_point_cloud(machine_path)
    # machine_pcd_initial = machine_pcd.voxel_down_sample(voxel_size=0.05)
    machine_pcd.paint_uniform_color([1, 0, 0])
    file = "/data/final_guards"
    pcds = data_manager.load_pcd_list(
        rospkg.RosPack().get_path('cleaning') + file)
    # Remove spots points
    spots = np.asarray(pcds[-1].points)
    del pcds[-1]



    '''
    # Get rotations
    
    
    # Add mir box
    for i in range(4):

        mir = o3d.geometry.TriangleMesh.create_box(0.75, 0.95, 0.40)
        controlleur = o3d.geometry.TriangleMesh.create_box(0.40, 0.50, 0.40)
        mir.paint_uniform_color([0, 0, 1])
        controlleur.paint_uniform_color([1, 0, 0])
        center_mir = mir.get_center()
        center_controlleur = controlleur.get_center()
        mir_t = copy.deepcopy(mir).translate((spots[i][0] - center_mir[0], spots[i][1] - center_mir[1], -0.1))
        controlleur_t = copy.deepcopy(controlleur).translate((spots[i][0] - center_controlleur[0], spots[i][1] - center_controlleur[1], -0.1 +0.4))

        R = mir_t.get_rotation_matrix_from_xyz((0, 0, spots[i][2]+1.57))
        mir_t.rotate(R)
        R = controlleur_t.get_rotation_matrix_from_xyz((0, 0, spots[i][2]))
        controlleur_t.rotate(R)

        pcds.append(mir_t)
        pcds.append(controlleur_t)

    o3d.visualization.draw_geometries(pcds)
    '''

    # Fuse the pcd
    final_pcd = o3d.geometry.PointCloud()
    somme = 0
    for pcd in pcds:
        somme += len(np.asarray(pcd.points))
        pcd_np = np.asarray(pcd.points)
        final_pcd_np = np.asarray(final_pcd.points)

        p3_load = np.concatenate((pcd_np, final_pcd_np), axis=0)
        final_pcd.points = o3d.utility.Vector3dVector(p3_load)
    final_pcd.paint_uniform_color([0, 0, 1])
    # o3d.visualization.draw_geometries([machine_pcd, final_pcd ])


    # Calcul porcentage correspondance
    dists = machine_pcd.compute_point_cloud_distance(final_pcd)
    dists = np.asarray(dists)
    ind = np.where(dists < 0.01)[0]

    print(file)
    print("Nb point pts observer:")
    print(len(np.asarray(final_pcd.points)))
    print("Nb point machine mesh:")
    print(len(np.asarray(machine_pcd.points)))
    print("Nb point en commun:")
    print(len(ind))
    result = machine_pcd.select_by_index(ind)
