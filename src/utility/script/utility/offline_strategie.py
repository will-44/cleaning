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
import time
# Debug
time_callback = 0
joint_time = time.time()
nb_joint_time = 0
global_time = 0
time_flag = True

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
        self.j1 = -180
        self.j2 =  -90
        self.j3 = -90
        self.j4 =  -180
        self.j5 =  -90
        self.j6 = 0

        # Actuel spot position
        self.spot_x = 0
        self.spot_y = 0
        self.spot_theta = 0

        # reposition the robot:
        self.mir_replace = 0

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
        # Initialiser un dictionnaire vide pour stocker les clefs et valeurs extraites
        extracted_values = {}

        # Itérer tant qu'il y a encore des éléments dans le dictionnaire
        while relation:

            # Trouver la clef avec la valeur la plus longue
            longest_key = max(relation, key=lambda key: len(relation[key]))

            # Extraire la valeur associée à la clef la plus longue
            longest_value = relation[longest_key]
            if not longest_value:
                break
            # Ajouter la clef et la valeur à notre dictionnaire extrait
            extracted_values[longest_key] = longest_value

            # Supprimer la clef et la valeur du dictionnaire original
            del relation[longest_key]

            # Pour chaque élément de la valeur de la clef la plus longue,
            # vérifier s'il se trouve dans les valeurs des autres clefs
            # et le supprimer si c'est le cas
            with alive_bar(len(longest_value) * len(relation.items())) as bar:
                for element in longest_value:
                    for key, value in relation.items():
                        bar()
                        if element in value:
                            value.remove(element)

            # print(extracted_values)

        # Put all spots in a pcd
        spots = [v for v in extracted_values.keys()]
        print(spots)
        spots = [(x, y, 1) for x, y in spots]
        print(spots)
        spots = self.np2pcd(np.asarray(spots))

        # Extracte all pcd match to the spot
        pcds_each_spot = []
        for key, value in extracted_values.items():
            color = [rnd.uniform(0.0, 1.0), rnd.uniform(0.0, 1.0), rnd.uniform(0.0, 1.0)]

            # Conversion en tableau NumPy
            pcd = self.np2pcd(np.array(value).reshape(len(value), 3))
            pcd.paint_uniform_color(color)
            pcds_each_spot.append(pcd)

        # Renvoyer le dictionnaire extrait
        return spots, pcds_each_spot

    def np2pcd(self, xyz):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    def callback_mir(self, msg):
        print("mir_result:", msg)
        self.mir_result = msg

    def callback_fov(self, msg):
        global time_callback
        # print("callback cone: ", ( time.time() - time_callback))
        # Add points to dict
        pcd = o3d_ros.rospc_to_o3dpc(msg)
        self.dict_pos2pts[(self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)] = np.asarray(pcd.points)
        
        joints, self.all_poses_check = self.increase_joint_angle()
        self.send_config_allow()
        time_callback =  time.time()

    def mir_pose_angle(self, pose_2d, pcd_associate):
        """ TEST
        Get the 2d pose and return the orientation referential of the machine
        :param pose_2d:
        :param pcd_associate: open3D pcd
        :return: orientation in radian
        """
        # get 2d pose of all pcd points
        pcd_associate_np = np.asarray(pcd_associate.points)
        # mean thoses poses x, y
        x = pcd_associate_np[:, 0]
        y = pcd_associate_np[:, 1]
        x_mean = np.mean(x)
        y_mean = np.mean(y)

        # get the orientation 2D with the atan
        a = pose_2d[0] - x_mean
        b = pose_2d[1] - y_mean
        angle = atan2(b, a) + np.pi

        return angle

    def move_base(self, x, y, theta):
        self.spot_x = x
        self.spot_y = y
        self.spot_theta = theta
        pose_mir = PoseStamped()
        pose_mir.header.frame_id = "machine"
        pose_mir.pose.position.x = x
        pose_mir.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, theta)
        # TODO modifie the orientation
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
    def get_fov_transformation(self, pose):
        '''
       Return the TCP pose in the map frame
       :param pose:
       :return: translatio matrix  and is pose valide
       '''

        current_time = time.time()
        # Environ 7s a mieux
        tcp_map = self.arm.fkin(pose).pose_stamped[0]
        # print("fkin: ", (time.time() - current_time))
        quaternion = (
            tcp_map.pose.orientation.x,
            tcp_map.pose.orientation.y,
            tcp_map.pose.orientation.z,
            tcp_map.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        if radians(160) >= euler[2] >= radians(30) and \
                radians(160) >= euler[1] >= radians(20):
            current_time =time.time()
            check = self.arm.check_collision(pose)
            # print("collision: ", (time.time() - current_time))
            if check.valid:
                # print("coucou")
                self.pose_valid.append(pose)
                # transform the pose from the map to the base_footprint
                try:
                    trans_machine = self.tf_buffer.lookup_transform('machine', 'map', rospy.Time(),
                                                                    rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    rospy.loginfo("pb dans la transformation")
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

                print(cone_transform[0])

                return cone_transform[0], True, tcp_machine
        return [0, 0, 0], False, [0, 0, 0]

    def increase_joint_angle(self):
        global joint_time, nb_joint_time, global_time, time_flag
        if time_flag:
            joint_time = time.time()
            time_flag = False
            global_time += time.time() - joint_time
            nb_joint_time += 1
        else:
            global_time += time.time() - joint_time
            nb_joint_time += 1
        print("mean time:", global_time / nb_joint_time)#sum(global_time) / len(global_time))

        # check if the mir flage is up to replace the robot

        self.mir_replace += 1
        if self.mir_replace >= 100:
            self.move_base(self.spot_x, self.spot_y, self.spot_theta)
            self.mir_replace = 0
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
                        if self.j1 > 180:
                            self.j1 = -180
                            all_poses_check = True
        # print(self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)
        joint_time = time.time()
        return [radians(self.j1), radians(self.j2), radians(self.j3),
                radians(self.j4), radians(self.j5), radians(self.j6)], all_poses_check

    def send_config_allow(self):
        if self.all_poses_check:
            print("all poses check")
            return
        matrix, is_valid, tcp_machine = self.get_fov_transformation([self.j1, self.j2, self.j3,
                                                                     self.j4, self.j5, self.j6])
        while not is_valid:
            # add Joints
            joints, all_poses_check = self.increase_joint_angle()
            if self.all_poses_check:
                print("all poses check in while")
                return
            matrix, is_valid, tcp_machine = self.get_fov_transformation(joints)

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
        print("pose find and send")
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
            b = len(dist_mat)
            if (len(nb_block_guard) >= (len(dist_mat) - 1)):
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

    def generate_trajectory(self, dist_mat):
        permutation, distance = solve_tsp_dynamic_programming(dist_matrix)
        print("permutation")
        print(permutation)
        return permutation


if __name__ == '__main__':
    rospy.init_node('offline_strategie', anonymous=True)

    pcd_path = rospkg.RosPack().get_path('utility') + "/mesh/scie1.ply"
    path_relation = rospkg.RosPack().get_path('utility') + "/data/relation.pkl"

    offline = OfflineStrategy(pcd_path, path_relation)

    # Get spots for the MiR
    # best_spots, pcds_spots = offline.select_best_spot(offline.relation, offline.pcd_machine)
    # print("best spot:")
    # print(best_spots)
    # input()
    # o3d.visualization.draw_geometries([offline.pcd_machine, pcds_spots[0], pcds_spots[1], pcds_spots[2], pcds_spots[3]])
    # best_spots_np = np.asarray(best_spots.points)

    # Init the space
    # offline.arm.go_to_j([0, 0, 0, 0, 0, 0])
    print("move arm:")
    # while offline.arm.check_motion() != 0:
    #     rospy.sleep(0.1)
    offline.arm.add_machine_colision(rospkg.RosPack().get_path('utility') + "/mesh/scie1.obj")

    #  Change the TCP for the compute
    offline.arm.set_tcp("camera_color_frame")
    rospy.sleep(5)
    print("move MiR:")
    best_spots_np = np.array([[1.09341357, 1.6552466]])
    # Send MiR to spots
    for index, spot in enumerate(best_spots_np):
        print(spot[0], spot[1], -1.57)#offline.mir_pose_angle(spot, pcds_spots[index]))
        offline.move_base(spot[0], spot[1], -1.57)#offline.mir_pose_angle(spot, pcds_spots[index]))

        # Get all guards
        offline.send_config_allow()

        # While all poses check sleep
        while not offline.all_poses_check:
            rospy.sleep(1)

        # Select best guards
        best_guard, guard_spots = offline.select_best_spot(offline.dict_pos2pts, offline.pcd_machine)
        best_guard_np = np.asarray(best_guard.points)
        print("best_guard_np:")
        print(best_guard_np)
        # Check guards accessibility
        dist_matrix = euclidean_distance_matrix(best_guard_np)
        dist_matrix = offline.check_obs_btw_guards(dist_matrix, best_guard_np)
        is_block, pts_block = offline.is_guard_isolate(dist_matrix)

        # Generate path
        permutation = offline.generate_trajectory(dist_matrix)

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

    #  Change the TCP at the end of the compute
    offline.arm.set_tcp("tcp")

    print("ok")
