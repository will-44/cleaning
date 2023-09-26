#!/usr/bin/env python3

import pickle

import geometry_msgs.msg
import numpy as np

import rospy
import tf2_geometry_msgs
import tf2_ros

from open3d_ros_helper import open3d_ros_helper as o3d_ros
from ros_numpy import numpify

from open3d_tools import Open3dTool
from robot import Robot
from utility.srv import DetectDust
from python_tsp.distances import euclidean_distance_matrix


class Dust:
    def __init__(self):
        self.o3d_tool = Open3dTool()
        self.robot = Robot()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        print("init dust")

    def ask_dust_poses(self, color_image, depth_image):
        """
        retrun a list a positions in the vacuum_tcp frame
        """
        rospy.wait_for_service('/detect_dust', timeout=5)
        result_poses = []
        try:
            detect_dust = rospy.ServiceProxy('/detect_dust', DetectDust)

            resp1 = detect_dust(color_image, depth_image)

            pcd_dust = o3d_ros.rospc_to_o3dpc(resp1.pcds[0])
            # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
            dust_poses = np.asarray(pcd_dust.points)

            # Transform to robot frame
            frame = resp1.pcds[0].header.frame_id
            try:
                trans_camera = self.tf_buffer.lookup_transform('vacuum_tcp', frame, rospy.Time(),
                                                          # why not direct the world frame ?
                                                          rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.loginfo('{nodeName} : Aucun message de la transformation'.format(nodeName=rospy.get_name()))
            for pt in dust_poses:
                pose = geometry_msgs.msg.PoseStamped()
                pose.pose.position.x = pt[0]
                pose.pose.position.y = pt[1]
                pose.pose.position.z = pt[2]
                pose.pose.orientation.w = 1.0
                pose.header.frame_id = frame

                pose = tf2_geometry_msgs.do_transform_pose(pose, trans_camera)

                result_poses.append(numpify(pose.pose.position)[:3])

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        print(result_poses)
        return result_poses

    def filter_dust(self, points, dist=0.4):
        """
        get points list in stampted points and return the same
        """
        # print(points)
        np_points = []
        for point in points:
            # point = numpify(point.pose.position)
            np_points.append([point[0], point[1], point[2]])

        np_points = np.asarray(np_points)
        # print(np_points)
        norm = np.linalg.norm(np_points, axis=1)
        # print(norm)
        selected_points = np_points[norm < dist]
        return selected_points

    def dust_path(self, dust_list):
        dust_path = []
        # dust_path = np.asarray(dust_path)
        if len(dust_list) >= 1:
            clusters = self.o3d_tool.clusterise_dbscan(dust_list, eps=0.1, neighbors=1)
            # print(clusters)
            raw = []
            # raw = np.asarray(raw)
            for cluster in clusters:
                for index in cluster:
                    raw.append(dust_list[index].tolist())
                dust_path.append(raw)
                # print(dust_path)
                raw = []
                # raw = np.array(raw)
            # Do the TSP between the fisrt point from dust cluster
            dust_path = np.array(dust_path)
            # print(dust_path)
            # print(type(dust_path))
            # entry_points = dust_path[:, 0]
            entry_points = [np.array(sublist[0]) for sublist in dust_path]
            # print(entry_points)
            # input("test:")
            if len(entry_points) != 1:
                # Do the TSP
                dist_matrix = ((euclidean_distance_matrix(entry_points)) * 1000).astype(int)
                dust_permutation = self.robot.generate_trajectory(dist_matrix, 2)
                dust_path = dust_path[dust_permutation]


        return entry_points, dust_path
