#!/usr/bin/env python3

import pickle

import geometry_msgs.msg
import numpy as np
import rospkg
import rospy
import tf2_geometry_msgs
import tf2_ros
from scipy.spatial.transform import Rotation as R
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



class Positionning:
    def __init__(self):
        self.artag = rospy.get_param("ar_tag")

        return

    def get_artag_position(self):
        # get artag from world
        # try:
        #     trans_machine2artag = self.tf_buffer.lookup_transform(self.artag, "machine", rospy.Time(),
        #                                                           rospy.Duration(1.0))
        # except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.loginfo("pb dans la transformation")
        # mat_surface2artag = numpify(trans_machine2artag.transform)
        try:
            trans_artag2base = self.tf_buffer.lookup_transform("base_0", "machine", rospy.Time(),
                                                                  rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")
        mat_machine2base = numpify(trans_artag2base.transform)
        rot = R.from_matrix(mat_machine2base)
        angle = rot.as_euler('xyz', degrees=False)
        rospy.loginfo("x: %f, y: %f, theta: %f", mat_machine2base[0][2], mat_machine2base[1][2], angle[2])
        return
