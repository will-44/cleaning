#!/usr/bin/env python3

from math import atan2

import fast_tsp
import geometry_msgs.msg
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from open3d_tools import Open3dTool
from visualization_msgs.msg import Marker
from utility.rviz_tool import display_marker, display_marker_array
import tf2_geometry_msgs


class Robot:

    def __init__(self):
        self.broadcaster_robot = tf2_ros.StaticTransformBroadcaster()
        # Publisher
        self.mir_pub = rospy.Publisher('/mir_go_to', PoseStamped, queue_size=10)
        self.spot_pub = rospy.Publisher('/telemetrie/spot', PoseStamped, queue_size=10)
        # Subscribers
        self.mir_sub = rospy.Subscriber("/mir_result", Bool, self.callback_mir)

        # Global var
        self.mir_result = False
        self.mir_did_answer = False
        self.debug = rospy.get_param("/debug", default=False)
        self.open3d_tool = Open3dTool()

    def callback_mir(self, msg):
        self.mir_result = msg.data
        self.mir_did_answer = True

    def move_mobile_base(self, x, y, theta):
        """
        Set the tf position of the mobile base
        :param x: position
        :param y: position
        :param theta: angle
        :return:
        """
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        pose = PoseStamped()
        pose.header.frame_id = "machine"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = (0.76 - 0.1)  # TODO change
        quaternion = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        # send tf
        static_transform_stamped = geometry_msgs.msg.TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = "machine"
        static_transform_stamped.child_frame_id = "world"
        static_transform_stamped.transform.translation.x = pose.pose.position.x
        static_transform_stamped.transform.translation.y = pose.pose.position.y
        static_transform_stamped.transform.translation.z = pose.pose.position.z
        static_transform_stamped.transform.rotation.x = pose.pose.orientation.x
        static_transform_stamped.transform.rotation.y = pose.pose.orientation.y
        static_transform_stamped.transform.rotation.z = pose.pose.orientation.z
        static_transform_stamped.transform.rotation.w = pose.pose.orientation.w

        self.broadcaster_robot.sendTransform(static_transform_stamped)
        # # Telemetrie
        try:
            trans_mir = tf_buffer.lookup_transform('world', "machine", rospy.Time(),
                                                   rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo('{nodeName} : Aucun message de la transformation'.format(nodeName=rospy.get_name()))
        tcp_pose_world = tf2_geometry_msgs.do_transform_pose(pose, trans_mir)
        euler = euler_from_quaternion([tcp_pose_world.pose.orientation.x, tcp_pose_world.pose.orientation.y,
                                       tcp_pose_world.pose.orientation.z, tcp_pose_world.pose.orientation.w])

        quaternion = quaternion_from_euler(0, 0, euler[2])

        tcp_pose_world.pose.orientation.x = quaternion[0]
        tcp_pose_world.pose.orientation.y = quaternion[1]
        tcp_pose_world.pose.orientation.z = quaternion[2]
        tcp_pose_world.pose.orientation.w = quaternion[3]
        tcp_pose_world.pose.position.z = 0
        self.spot_pub.publish(tcp_pose_world)
        # return self.mir_result
        return True, trans_mir

    def move_base(self, x, y, theta):
        """
        Send the goal to the mobile base
        :param x: position
        :param y: position
        :param theta: angle
        :return:
        """
        pose_mir = PoseStamped()
        pose_mir.header.frame_id = "machine"
        pose_mir.pose.position.x = x
        pose_mir.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, theta)
        pose_mir.pose.orientation.x = quaternion[0]
        pose_mir.pose.orientation.y = quaternion[1]
        pose_mir.pose.orientation.z = quaternion[2]
        pose_mir.pose.orientation.w = quaternion[3]
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            trans_mir = tf_buffer.lookup_transform('map', "machine", rospy.Time(),
                                                   rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo('{nodeName} : Aucun message de la transformation'.format(nodeName=rospy.get_name()))
        tcp_pose_world = tf2_geometry_msgs.do_transform_pose(pose_mir, trans_mir)
        euler = euler_from_quaternion([tcp_pose_world.pose.orientation.x, tcp_pose_world.pose.orientation.y,
                                       tcp_pose_world.pose.orientation.z, tcp_pose_world.pose.orientation.w])

        quaternion = quaternion_from_euler(0, 0, euler[2])

        tcp_pose_world.pose.orientation.x = quaternion[0]
        tcp_pose_world.pose.orientation.y = quaternion[1]
        tcp_pose_world.pose.orientation.z = quaternion[2]
        tcp_pose_world.pose.orientation.w = quaternion[3]
        tcp_pose_world.pose.position.z = 0
        print(tcp_pose_world)

        display_marker(Marker.ARROW, x, y, 0, quaternion[0], quaternion[1], quaternion[2], quaternion[3], "machine")
        # print(pose_mir)
        self.mir_pub.publish(tcp_pose_world)
        rospy.sleep(1)
        rospy.loginfo('{nodeName} : Message envoyer a la base mobile'.format(nodeName=rospy.get_name()))
        tcp_pose_world.header.stamp = rospy.get_rostime()
        while not self.mir_did_answer:
            rospy.sleep(0.1)
        self.mir_did_answer = False

        # Telemetrie
        self.spot_pub.publish(tcp_pose_world)
        return self.mir_result

    def mir_pose_angle(self, pose_2d, pcd):
        """
        Get the 2d pose and return the orientation referential of the machine
        :param pose_2d:
        :param pcd: open3D pcd
        :return: orientation in radian
        """
        center = self.open3d_tool.get_pcd_density_center(pcd)

        # get the orientation 2D with the atan
        a = pose_2d[0] - center[0]
        b = pose_2d[1] - center[1]
        angle = atan2(b, a) + np.pi

        return angle

    def generate_trajectory(self, dist_mat, duration_seconds=20):
        """
        Generate a trajectorie with a TSP algorithm
        :param duration_seconds:
        :param dist_mat:
        :return:
        """
        perm = fast_tsp.find_tour(dist_mat, duration_seconds)
        cost_solver = fast_tsp.compute_cost(perm, dist_mat)
        tour = fast_tsp.greedy_nearest_neighbor(dist_mat)
        cost_greedy = fast_tsp.compute_cost(tour, dist_mat)
        if True:
            print("cost for find tour:")
            print(cost_solver)
            print("cost for greedy nearest neighbor:")
            print(cost_greedy)
        res = []
        # res = perm
        if cost_greedy >= cost_solver:
            res = perm
        else:
            res = tour
        return res

    def sort_poses(self, poses, order):
        """
        Set the poses in the correct order
        :param poses: list of poses
        :param order: list of order
        :return: list of poses sort
        """
        result = np.zeros((len(order), len(poses[0])))
        for index_loop, index_value in enumerate(order):
            result[index_loop] = poses[index_value]
        return result

    def joint_tsp(self, configurations, robot_speeds):
        '''

        :param configurations: the array of guards configurations
        :param robot_speeds: the speed of all joints
        :return: The list of index order
        '''
        distances = np.zeros((len(configurations), len(configurations)))
        for i, initial_config in enumerate(configurations):
            for j, compare_config in enumerate(configurations):
                diff = np.max(np.abs(initial_config - compare_config) / robot_speeds)
                distances[i, j] = diff
        return distances
