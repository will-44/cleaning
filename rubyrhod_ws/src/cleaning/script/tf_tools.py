#!/usr/bin/env python3

import pickle

import geometry_msgs.msg
import numpy as np
import rospkg
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose, Pose2D
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
from scipy.spatial.transform import Rotation as R
from data_manager import DataManager
from open3d_tools import Open3dTool
from robot import Robot
# from dust import Dust
from utility.srv import DetectDust
from python_tsp.distances import euclidean_distance_matrix


class TfTools:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
    def get_transform(self, origin, target):
        try:
            trans_result = self.tf_buffer.lookup_transform(target, origin, rospy.Time(),
                                                   rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo(
                '{nodeName} : Aucun message de la transformation'.format(nodeName=rospy.get_name()))
        return trans_result

    def transform_pose_array(self, poses, transform):
        """
        get a poseStamped array and return a poseStamped array transformed
        :param poses: PoseStamped array
        :param transform:
        :return: array of PoseStamped transformed
        """
        # if type(poses) != list:
        #     raise TypeError('poses is not a PoseStamped but a : ', type(poses))
        result = []
        for pose in poses:
            pose_trans = tf2_geometry_msgs.do_transform_pose(pose, transform)
            result.append(pose_trans)
        return result

    def array_2_posestamped(self, list):
        """
        get a array of 3d poses and retrun a array of posestamped
        :param list:
        :return:
        """
        result = []
        for pose in list:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = pose[2]
            result.append(pose_stamped)
        return result





