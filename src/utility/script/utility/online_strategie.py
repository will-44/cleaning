#!/usr/bin/env python3

import rospy
from PIL.ImageChops import offset
from ros_numpy import numpify
import rospkg
from doosan import Doosan
from robotiq import Robotiq
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose2D, Pose
from visualization_msgs.msg import Marker
from rviz_tool import display_marker
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
from sensor_msgs.msg import Image, CameraInfo

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


from utility.srv import *


class OnlineStrategy:
    def __init__(self, path_machine, offline_trajectory_path):
        # Callback ROS
        self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback_color)
        self. depth_sub = rospy.Subscriber("/depth_to_rgb/image_raw", Image, self.callback_depth)
  
        # Publisher 
        self.pcl_pub = rospy.Publisher('/transf_mat', Float64MultiArray, queue_size=10)


        # Dicts
        with open(offline_trajectory_path, 'rb') as f:
            self.offline_traj = pickle.load(f)

        # Global var
        self.base_trajectory = self.sort_poses(spot_poses,self.generate_base_traj(list(map(list,list(self.offline_traj.keys())))))
        # print(self.base_trajectory)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.broadcaster_robot = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.j1 = 0
        self.j2 = 0
        self.j3 = 0
        self.j4 = 0
        self.j5 = 0
        self.j6 = 0
        
        

        self.arm = Doosan()
        print("online ready")

    def callback_color(self, img):
        global color_image
        color_image = img

    def callback_depth(self, img):
        global depth_image
        depth_image = img


    def ask_dust_poses(self):

        rospy.wait_for_service('/detect_dust', timeout=5)
        result_poses = []
        try:
            detect_dust = rospy.ServiceProxy('/detect_dust', DetectDust)
            
            resp1 = detect_dust(color_image, depth_image)
            
            pcd_dust = o3d_ros.rospc_to_o3dpc(resp1.pcds[0])
            # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
            dust_poses = np.asarray(pcd_dust.points)

            # TSP
            dust_permutation = self.generate_base_traj(dust_poses)
            
            # Sort the array
            result_poses = self.sort_poses(dust_poses, dust_permutation)
            

            # Transform to robot frame
            frame = resp1.pcds[0].header.frame_id
            try:
                trans_camera = tf_buffer.lookup_transform('vacuum_tcp', frame, rospy.Time(),
                                                        rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.loginfo("pb dans la transformation")
            for pt in dust_poses:
                pose = geometry_msgs.msg.PoseStamped()
                pose.pose.position.x = pt[0]
                pose.pose.position.y = pt[1]
                pose.pose.position.z = pt[2]
                pose.pose.orientation.w = 1.0
                pose.header.frame_id = frame

                pose = tf2_geometry_msgs.do_transform_pose(pose, trans_camera)

                result_poses.append(pose)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        return result_poses

    def send_base(self):

        return True
     
    def generate_path(self):

        return True

    def generate_base_traj(self, spot_poses):
        dist_matrix = ((euclidean_distance_matrix(spot_poses))*1000).astype(int)
        permutation = fast_tsp.find_tour(dist_matrix, duration_seconds=5)
        return permutation

    def sort_poses(self, poses, order):
        result = np.zeros((len(order), len(poses[0])))
        for index, index_value in enumerate(order):
            # print(index_value)
            # print(poses[index])
            result[index_value] = poses[index]
        return result

    def move_robot(self, x, y, theta):
        self.spot_x = x
        self.spot_y = y
        self.spot_theta = theta
        pose = PoseStamped()
        pose.header.frame_id = "machine"
        pose.pose.position.x = x
        pose.pose.position.y = y
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
        static_transform_stamped.transform.translation.z = 0.76 +0.2
        static_transform_stamped.transform.rotation.x = pose.pose.orientation.x
        static_transform_stamped.transform.rotation.y = pose.pose.orientation.y
        static_transform_stamped.transform.rotation.z = pose.pose.orientation.z
        static_transform_stamped.transform.rotation.w = pose.pose.orientation.w

        self.broadcaster_robot.sendTransform(static_transform_stamped)

    def move_base(self, x, y, theta):
        self.spot_x = x
        self.spot_y = y
        self.spot_theta = theta
        pose_mir = PoseStamped()
        pose_mir.header.frame_id = "machine"
        pose_mir.pose.position.x = x
        pose_mir.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, theta)
        pose_mir.pose.orientation.x = quaternion[0]
        pose_mir.pose.orientation.y = quaternion[1]
        pose_mir.pose.orientation.z = quaternion[2]
        pose_mir.pose.orientation.w = quaternion[3]
        display_marker(Marker.ARROW, x, y, 0, quaternion[0], quaternion[1], quaternion[2], quaternion[3], "machine")
        self.mir_pub.publish(pose_mir)
        rospy.loginfo("pose send")
        while not self.mir_result:
            rospy.sleep(0.1)
        self.mir_result = False
    
    def get_fov_transformation(self, pose, trans_machine):
        '''
       Return the TCP pose in the map frame
       :param pose: joint pose
       :return: translatio matrix  and is pose valide
       '''

        current_time = time.time()
        # Environ 7s au mieux
        tcp_map = self.arm.fkin(pose).pose_stamped[0]
        # print("fkin: ", (time.time() - current_time))
        quaternion = (
            tcp_map.pose.orientation.x,
            tcp_map.pose.orientation.y,
            tcp_map.pose.orientation.z,
            tcp_map.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        current_time =time.time()
        check = self.arm.check_collision(pose)
        
      
        self.j1 = pose[0]
        self.j2 = pose[1]
        self.j3 = pose[2]
        self.j4 = pose[3]
        self.j5 = pose[4]
        self.j6 = pose[5]
        # rospy.loginfo("collision free")
        # self.pose_valid.append(pose)
        # transform the pose from the map to the base_footprint

        tcp_machine = tf2_geometry_msgs.do_transform_pose(tcp_map, trans_machine)
        transformer = tf.TransformerROS()
        # Get transform matrix
        cone_transform = transformer.fromTranslationRotation((tcp_machine.pose.position.x,
                                                                tcp_machine.pose.position.y,
                                                                tcp_machine.pose.position.z),
                                                                (
                                                                    tcp_machine.pose.orientation.x,
                                                                    tcp_machine.pose.orientation.y,
                                                                    tcp_machine.pose.orientation.z,
                                                                    tcp_machine.pose.orientation.w))
        display_marker(Marker.CUBE,
                        tcp_machine.pose.position.x,
                        tcp_machine.pose.position.y,
                        tcp_machine.pose.position.z,
                        tcp_machine.pose.orientation.x,
                        tcp_machine.pose.orientation.y,
                        tcp_machine.pose.orientation.z,
                        tcp_machine.pose.orientation.w,
                        "machine")
        cone_transform.resize(1, 16)

        

        return cone_transform[0], tcp_machine
        
  
    def send_cone_pose(self, matrix, tcp_machine ):
        # send tf
        static_transform_stamped = geometry_msgs.msg.TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = "machine"
        static_transform_stamped.child_frame_id = "cone"
        static_transform_stamped.transform.translation.x = tcp_machine.pose.position.x
        static_transform_stamped.transform.translation.y = tcp_machine.pose.position.y
        static_transform_stamped.transform.translation.z = tcp_machine.pose.position.z
        static_transform_stamped.transform.rotation.x = tcp_machine.pose.orientation.x
        static_transform_stamped.transform.rotation.y = tcp_machine.pose.orientation.y
        static_transform_stamped.transform.rotation.z = tcp_machine.pose.orientation.z
        static_transform_stamped.transform.rotation.w = tcp_machine.pose.orientation.w

        # self.broadcaster.sendTransform(static_transform_stamped)

        # send matrix to node
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())
        msg.data = matrix
        msg.layout.dim[0].label = "height"
        msg.layout.dim[0].size = 4
        msg.layout.dim[0].stride = 16
        msg.layout.dim[1].label = "width"
        msg.layout.dim[1].size = 4
        msg.layout.dim[1].stride = 4
        print("pose find and send")
        self.pcl_pub.publish(msg)



if __name__ == '__main__':
    global color_image, depth_image
    rospy.init_node('online_strategie', anonymous=True)
    
    
    traj_path = rospkg.RosPack().get_path('utility') + '/data/offline_trajectory.pkl'
    machine_pcd_path = rospkg.RosPack().get_path('utility') + "/mesh/scie1.ply"
    
    broadcaster_guard = tf2_ros.StaticTransformBroadcaster()
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    strat = OnlineStrategy(machine_pcd_path, traj_path)
    # rospy.sleep(1)
    strat.arm.set_tcp("vacuum_tcp")

    
        
    dust_poses = strat.ask_dust_poses()
    for dust in dust_poses: 
        tcp_pose = strat.arm.get_pose()

        try:
            trans_tcp = tf_buffer.lookup_transform('vacuum_tcp', "world", rospy.Time(),
                                                    rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")
        tcp_pose_world = tf2_geometry_msgs.do_transform_pose(tcp_pose, trans_tcp)

        print(tcp_pose_world)
        pose = tcp_pose_world
        pose.pose.position.x = dust.pose.position.x
        pose.pose.position.y = dust.pose.position.y
        pose.pose.position.z = dust.pose.position.z


        try:
            trans_world = tf_buffer.lookup_transform('world', "vacuum_tcp", rospy.Time(),
                                                    rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")
        goal_pose = tf2_geometry_msgs.do_transform_pose(pose, trans_world)
        
        # send tf
        static_transform_stamped = geometry_msgs.msg.TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = "world"
        static_transform_stamped.child_frame_id = "goal"
        static_transform_stamped.transform.translation.x = goal_pose.pose.position.x
        static_transform_stamped.transform.translation.y = goal_pose.pose.position.y
        static_transform_stamped.transform.translation.z = goal_pose.pose.position.z
        static_transform_stamped.transform.rotation.x = goal_pose.pose.orientation.x
        static_transform_stamped.transform.rotation.y = goal_pose.pose.orientation.y
        static_transform_stamped.transform.rotation.z = goal_pose.pose.orientation.z
        static_transform_stamped.transform.rotation.w = goal_pose.pose.orientation.w

        broadcaster_guard.sendTransform(static_transform_stamped)



        strat.arm.go_to_l(goal_pose.pose)
        input("next ?")
        strat.arm.go_to_l(tcp_pose.pose)

    # for spot in strat.base_trajectory:
        
    #     strat.move_robot(spot[0], spot[1], spot[2])

    #     tf_buffer = tf2_ros.Buffer()
    #     listener = tf2_ros.TransformListener(tf_buffer)

    #     try:
    #         trans_machine = tf_buffer.lookup_transform('machine', 'world', rospy.Time(),
    #                                                 rospy.Duration(10.0))
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
    #             tf2_ros.ExtrapolationException):
    #         rospy.loginfo("pb dans la transformation")


    #     strat.arm.add_machine_colision(rospkg.RosPack().get_path('utility') + "/mesh/scie1.obj")
    #     # rospy.sleep(10)

    #     arm_poses = strat.offline_traj[tuple(spot)]
    #     strat.arm.set_tcp("camera_base")
        
    #     for index, pose in enumerate(arm_poses):
    #         # arm.go_home()
    #         fkin = strat.arm.fkin(pose).pose_stamped[0]
    #         # print(fkin.header.frame_id)
    #         tcp_pose = fkin.pose
    #         a, b = strat.get_fov_transformation(pose , trans_machine)
    #         strat.send_cone_pose(a, b)


    #         # Transform pose from camera to doosan ref
    #         # try:
    #         #     transform = tf_buffer.lookup_transform('camera_link', 'world', rospy.Time(),
    #         #                                                     rospy.Duration(1.0))
    #         # except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
    #         #         tf2_ros.ExtrapolationException):
    #         #     rospy.loginfo("pb dans la transformation")
    #         # tcp_pose = tf2_geometry_msgs.do_transform_pose(tcp_pose, transform)


    #         # send tf
    #         static_transform_stamped = geometry_msgs.msg.TransformStamped()
    #         static_transform_stamped.header.stamp = rospy.Time.now()
    #         static_transform_stamped.header.frame_id = "world"
    #         static_transform_stamped.child_frame_id = "guard"
    #         static_transform_stamped.transform.translation.x = tcp_pose.position.x
    #         static_transform_stamped.transform.translation.y = tcp_pose.position.y
    #         static_transform_stamped.transform.translation.z = tcp_pose.position.z
    #         static_transform_stamped.transform.rotation.x = tcp_pose.orientation.x
    #         static_transform_stamped.transform.rotation.y = tcp_pose.orientation.y
    #         static_transform_stamped.transform.rotation.z = tcp_pose.orientation.z
    #         static_transform_stamped.transform.rotation.w = tcp_pose.orientation.w

    #         broadcaster_guard.sendTransform(static_transform_stamped)



    #         strat.arm.go_to_l(tcp_pose)

    #         while strat.arm.check_motion() != 0:
    #             rospy.sleep(0.1)

    #         rospy.sleep(5)

    # #     pose_goal = geometry_msgs.msg.Pose()



    # #     arm.go_to_l(pose_goal)

    # # look at

    # # get dust poses

    # # remove non valide poses

    # # create TSP 

    # # Send movements

    