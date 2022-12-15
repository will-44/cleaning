#!/usr/bin/env python3

import rospy
from PIL.ImageChops import offset
from ros_numpy import numpify
import rospkg
from doosan import Doosan
from robotiq import Robotiq
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose2D
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

from tf.transformations import euler_from_quaternion, quaternion_from_euler, \
    quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2
from open3d_ros_helper import open3d_ros_helper as o3d_ros

from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.distances import great_circle_distance_matrix, euclidean_distance_matrix



class OfflineStrategy:
    def __init__(self, path_machine, relation_path):
        """

        :param path_machine:
        :param relation_path:
        """
        self.path_machine = path_machine
        self.pcd_machine = o3d.io.read_point_cloud(path_machine)
        self.mesh_machine = o3d.io.read_triangle_mesh(path_machine)
        self.relation_path = relation_path
        with open(self.relation_path, 'rb') as f:
            self.relation = pickle.load(f)

        # robot
        self.arm = Doosan()

        # Joint poses
        self.j1 = -150
        self.j2 = -60 #-90
        self.j3 = 10#-90
        self.j4 = -180 #-180
        self.j5 = 50 #-90
        self.j6 = 0

        # Guard dictionnary
        self.dict_pos2pts = {}

        # Subscribers
        self.mir_sub = rospy.Subscriber("/mir_result", Bool, self.callback_mir)
        self.pcd_sub = rospy.Subscriber("/pcl_result", PointCloud2, self.callback_fov)

        # Publisher
        self.mir_pub = rospy.Publisher('/mir_go_to', PoseStamped, queue_size=10)
        self.pcl_pub = rospy.Publisher('/transf_mat', Float64MultiArray, queue_size=10)

        # Flag for subsciber callback
        self.mir_result = False
        self.all_poses_check = False
        self.all_poses_check = False

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        # ??????
        self.pose_valid = []


    def select_best_spot(self, relation, pcd_inital):
        """

        :param relation: Dictionnary key poses (2D or 3D) values pcd
        :param pcd_inital: the pcd of the machine
        :return: pcd_spot: a pcd of all best spots, spot_points: a list of pcd associated to each spot
        """
        flag_first = True
        list_spot = {}
        relation_sort = sorted(relation, key=lambda a: len(relation[a]), reverse=True)
        # Arbitraire
        max_len = len(np.asarray(pcd_inital.points)) * 0.50

        # Debug
        plt.axis([0, 600, 0, 20000])

        loop_index = 0
        nb_pt_covered = 0
        while nb_pt_covered <= max_len:
            loop_index += 1

            if flag_first:
                list_spot.update({relation_sort[0]: relation[relation_sort[0]]})
                nb_pt_covered += len(relation[relation_sort[0]])
                self.relation.pop(relation_sort[0])
                flag_first = False
                relation_sort = sorted(self.relation, key=lambda a: len(relation[a]), reverse=True)
                continue

            # on check tout les pts trouve jusqua present
            commun_pts = 0
            is_valid = True
            for pt in list_spot:
                inter_pts = set(relation[relation_sort[0]]).intersection(list_spot[pt])
                commun_pts = commun_pts + len(inter_pts)
            if commun_pts >= 1150:
                #
                is_valid = False
            # Debug
            plt.scatter(loop_index, nb_pt_covered)
            # if we check 80% pts in the dict it should be ok
            if loop_index >= 500:
                break
            # Debug
            plt.pause(0.005)

            if is_valid:
                # Delete all pts in commun
                diff = relation[relation_sort[0]]
                for pt in list_spot:
                    diff = tuple(map(tuple, set(diff).difference(list_spot[pt])))
                relation.update({relation_sort[0]: diff})
                # on prend ce pts
                nb_pt_covered += len(relation[relation_sort[0]])
                list_spot.update({relation_sort[0]: relation[relation_sort[0]]})
                relation.pop(relation_sort[0])
            else:
                # On update cet emplacement dans la relation pour toute les pos possible choisis
                diff = relation[relation_sort[0]]
                for pt in list_spot:
                    diff = tuple(map(tuple, set(diff).difference(list_spot[pt])))
                relation.update({relation_sort[0]: diff})
            relation_sort = sorted(relation, key=lambda a: len(relation[a]), reverse=True)

        best_spots = []
        spot_points = []
        for pt in list_spot.keys():
            pt = list(pt)
            pt.append(1)
            best_spots.append(pt)
        pcd_spot = self.np2pcd(best_spots)
        pcd_spot.paint_uniform_color([1, 0, 1])

        for i in range(len(pcd_spot.colors)):
            color = [rnd.uniform(0.0, 1.0), rnd.uniform(0.0, 1.0), rnd.uniform(0.0, 1.0)]
            pcd_spot.colors[i] = color
            spot_points.append(self.np2pcd(list(list_spot.values())[i]))

            dists = pcd_inital.compute_point_cloud_distance(spot_points[i])
            dists = np.asarray(dists)
            ind = np.where(dists < 0.1)[0]
            spot_points[i] = pcd_inital.select_by_index(ind)

            spot_points[i].paint_uniform_color(color)
        return pcd_spot, spot_points

    def np2pcd(self, xyz):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    def callback_mir(self, msg):
        print("mir_result:", msg)
        self.mir_result = msg

    def callback_fov(self, msg):
        # Add points to dict
        pcd = o3d_ros.rospc_to_o3dpc(msg)
        self.dict_pos2pts[(self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)] = np.asarray(pcd.points)

        joints, self.all_poses_check = self.increase_joint_angle()
        self.send_config_allow()


    def mir_pose_angle(self):
        print("DSF")

    def move_base(self, x, y, theta):
        pose_mir = PoseStamped()
        pose_mir.header.frame_id = "machine"
        pose_mir.pose.position.x = x
        pose_mir.pose.position.y = y
        # TODO modifie the orientation
        pose_mir.pose.orientation.x = 0
        pose_mir.pose.orientation.y = 0
        pose_mir.pose.orientation.z = -0.707
        pose_mir.pose.orientation.w = 0.707

        self.mir_pub.publish(pose_mir)
        rospy.loginfo("pose send")
        while not self.mir_result:
            rospy.sleep(0.1)

    def get_fov_transformation(self):
        print("DSF")
    def increase_joint_angle(self):
        all_poses_check = False
        # Add x degree to the joints
        self.j6 = 0
        self.j5 += 10
        if self.j5 > 90:
            self.j5 = -90
            self.j4 += 10
            if self.j4 > 180:
                self.j4 = -180
                self.j3 += 10
                if self.j3 > 90:
                    self.j3 = -90
                    self.j2 += 10
                    if self.j2 > 90:
                        self.j2 = -90
                        self.j1 += 10
                        if self.j1 > -30:
                            self.j1 = -150
                            all_poses_check = True
        print(self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)
        return [radians(self.j1), radians(self.j2), radians(self.j3),
                radians(self.j4), radians(self.j5), radians(self.j6)], all_poses_check
    def send_config_allow(self):
        if self.all_poses_check:
            print("all poses check")
            return
        matrix, is_valid, tcp_machine = self.get_cone_translation([self.j1, self.j2, self.j3,
                                                                   self.j4, self.j5, self.j6])
        while not is_valid:
            # add Joints
            joints, all_poses_check = self.increase_joint()
            if self.all_poses_check:
                print("all poses check in while")
                return
            matrix, is_valid, tcp_machine = self.get_cone_translation(joints)

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

        self.broadcaster.sendTransform(static_transform_stamped)

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

        self.pcl_pub.publish(msg)
    def is_guard_isolate(self, dist_mat):
        """
        Witch guards are isolate and unreachable
        :param dist_mat:
        :return:
        """

        pts_block = []
        is_guard_block = False
        for i in range(len(dist_mat)):
            nb_block_guard = np.argwhere(dist_mat[i, :] == 10000)
            a = len(nb_block_guard)
            b =len(dist_mat)
            if(len(nb_block_guard) >= (len(dist_mat)-1)):
                is_guard_block = True
                pts_block.append(i)

        return is_guard_block, pts_block

    def check_obs_btw_guards(self, dist_mat, guard_pts):
        """

        :param dist_mat:
        :param machine_mesh:
        :param guard_pts:
        :return:
        """

        scene = o3d.t.geometry.RaycastingScene()
        machine_mesh = o3d.t.geometry.TriangleMesh.from_legacy(self.mesh_machine)
        cube_id = scene.add_triangles(machine_mesh)

        for idx, x in np.ndenumerate(dist_mat):
            if idx[0] != idx[1]:
                # print(guard_pts[idx[0]], guard_pts[idx[1]])
                v = [guard_pts[idx[1]][0] - guard_pts[idx[0]][0],
                     guard_pts[idx[1]][1] - guard_pts[idx[0]][1],
                     guard_pts[idx[1]][2] - guard_pts[idx[0]][2]]
                v_unit = v / np.linalg.norm(v)
                rays = o3d.core.Tensor([[guard_pts[idx[0]][0], guard_pts[idx[0]][1], guard_pts[idx[0]][2],
                                         v_unit[0], v_unit[1], v_unit[2]]],
                                       dtype=o3d.core.Dtype.Float32)

                ans = scene.cast_rays(rays)
                # print(ans)
                if ans['t_hit'] != float('inf'):
                    dist_mat[idx[0], idx[1]] = 10000

        return dist_mat

    def generate_trajectory(self):
        print("ds")




if __name__ == '__main__':
    rospy.init_node('offline_strategie', anonymous=True)

    pcd_path = rospkg.RosPack().get_path('utility') + "/mesh/scie1.ply"
    path_relation = rospkg.RosPack().get_path('utility') + "/data/relation.pkl"

    offline = OfflineStrategy(pcd_path, path_relation)

    # Init the space
    offline.arm.go_to_j([0, 0, 0, 0, 0, 0])
    while offline.arm.check_motion() != 0:
        rospy.sleep(0.1)
    offline.arm.add_machine_colision(rospkg.RosPack().get_path('utility') + "/mesh/scie1.obj")

    # Get spots for the MiR
    best_spots, points_spots = offline.select_best_spot(offline.relation, offline.pcd_machine)
    best_spots_np = np.asarray(best_spots.points)

    # Send MiR to spots
    for spot in best_spots_np:
        offline.move_base(spot[0], spot[1], 0)

        # Get all guards
        offline.send_config_allow()

        # While all poses check sleep
        while not offline.all_poses_check:
            rospy.sleep(1)

        # Select best guards
        best_guard, guard_spots = offline.select_best_spot(offline.dict_pos2pts, offline.pcd_machine)
        best_guard_np = np.asarray(best_guard.points)
        # Check guards accessibility
        dist_matrix = euclidean_distance_matrix(best_guard_np)
        dist_matrix = offline.check_obs_btw_guards(dist_matrix, best_guard_np)
        is_block, pts_block = offline.is_guard_isolate(dist_matrix)

        # Generate path
        permutation, distance = solve_tsp_dynamic_programming(dist_matrix)
        
        # DEBUG
        lines = []
        for i in range(len(permutation)):
            lines.append([permutation[i], permutation[(i + 1) % len(permutation)]])

        colors = [[1, 0, 0] for i in range(len(lines))]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(best_guard_np)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([best_guard, offline.pcd_machine, line_set])




    print("ok")
