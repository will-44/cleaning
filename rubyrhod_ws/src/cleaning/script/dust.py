#!/usr/bin/env python3

import pickle

import geometry_msgs.msg
import numpy as np
import rospkg

import rospy
import tf2_geometry_msgs
import tf2_ros
import open3d as o3d

from open3d_ros_helper import open3d_ros_helper as o3d_ros
from ros_numpy import numpify

from open3d_tools import Open3dTool
from robot import Robot
from utility.srv import DetectDust
from python_tsp.distances import euclidean_distance_matrix

from tf_tools import TfTools
from video_article import VideoArticle

from sensor_msgs.msg import Image

class Dust:
    def __init__(self):
        self.o3d_tool = Open3dTool()
        self.robot = Robot()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_tools = TfTools()
        pcd_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_pcd")
        self.pcd_machine = o3d.io.read_point_cloud(pcd_path)
        self.video_article = VideoArticle()
        # Callback ROS
        self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback_color)
        self.depth_sub = rospy.Subscriber("/depth_to_rgb/image_raw", Image, self.callback_depth)

        self.color_image = None
        self.depth_image = None
        print("init dust")


    def callback_color(self, img):
        self.color_image = img

    def callback_depth(self, img):
        self.depth_image = img

    def ask_dust_poses(self):
        """
        retrun a list a positions in the vacuum_tcp frame
        """
        rospy.wait_for_service('/detect_dust', timeout=5)
        result_poses = []
        # self.color_image = color_image
        # self.depth_image = depth_image
        try:
            detect_dust = rospy.ServiceProxy('/detect_dust', DetectDust)

            resp1 = detect_dust(self.color_image, self.depth_image)

            pcd_dust = o3d_ros.rospc_to_o3dpc(resp1.pcds[0])
            # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
            dust_poses = np.asarray(pcd_dust.points)

            # Transform to robot frame
            frame = resp1.pcds[0].header.frame_id
            try:##changment targert frame from vacuum_tcp to link6
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
        # print(result_poses)
        return result_poses

    def filter_dust(self, dusts, dist=0.5):
        """
        get points list in stampted points and return the same
        :param dusts: array of
        :param dist:
        :return:
        """
        if (len(dusts) == 0):
            return []
        rejected_points = []
        # remove point to far from tcp
        dusts = np.asarray(dusts)
        norm = np.linalg.norm(dusts, axis=1)
        valid_dusts = dusts[norm < dist]

        # get rejected points
        rejected_points = dusts[norm > dist]
        #remove point to close from machine
        valid_dusts = valid_dusts[abs(valid_dusts[:, 2]) > 0.05]

        #add rejected points
        too_close = valid_dusts[abs(valid_dusts[:, 2]) < 0.05]
        rejected_points = np.concatenate((too_close, rejected_points), axis=0)

        self.video_article.points_to_picture(valid_dusts, rejected_points, self.color_image)




        return valid_dusts





    def dust_path(self, dust_list):
        """
        get a list of dust point and return a list of dust path with the entry point of each dust cluster
        :param dust_list: list of dust point
        :return: list of entry_point and list of dust path
        """
        dust_path = []
        if len(dust_list) >= 1:
            clusters, noise = self.o3d_tool.clusterise_dbscan(dust_list, eps=0.13, neighbors=1)
            # add the noise to the cluster list
            for index in noise:
                clusters.append([index])
            raw = []

            for cluster in clusters:
                for index in cluster:
                    raw.append(dust_list[index].tolist())
                dust_path.append(raw)
                raw = []
            # Do the TSP between the fisrt point from dust cluster
            dust_path = np.array(dust_path)
            # get the firts point of each cluster
            entry_points = [np.array(sublist[0]) for sublist in dust_path]
            if len(entry_points) > 1:
                # Do the TSP for entry points
                dist_matrix = ((euclidean_distance_matrix(entry_points)) * 1000).astype(int)
                dust_permutation = self.robot.generate_trajectory(dist_matrix, 2)
                # Sort the entry points from the tsp result
                print("entry_points:")
                print(entry_points)
                #entry_points = entry_points[dust_permutation]

                entry_points = self.robot.sort_poses(entry_points, dust_permutation)
                dust_path = dust_path[dust_permutation]
                print("dust_path:")
                print(dust_path)
                print("entry_points:")
                print(entry_points)
                # input("Press Enter to continue...")
        else:
            entry_points = []
            dust_path = []
        return entry_points, dust_path
