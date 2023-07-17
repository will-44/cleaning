#!/usr/bin/env python3
import random
import random as rnd

import numpy as np
import open3d as o3d
import rospkg
import rospy
import tf
import tf2_geometry_msgs
import tf2_ros
from alive_progress import alive_bar
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from open3d_ros_helper import open3d_ros_helper as o3d_ros
from python_tsp.distances import euclidean_distance_matrix
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf.transformations import euler_from_quaternion
from utility.doosan import Doosan
from utility.rviz_tool import display_marker
from visualization_msgs.msg import Marker

from data_manager import DataManager
from open3d_tools import Open3dTool
from robot import Robot
from class_fov_emulator import FovEmulator

import matplotlib.pyplot as plt

import math


class Offline:

    def __init__(self, path_machine, path_mesh, relation_path, debug=False):

        self.data_manager = DataManager()
        self.open3d_tool = Open3dTool()
        self.robot = Robot()
        self.fov_emulator = FovEmulator()
        self.debug = debug

        # PCD and Mesh of the machine
        self.pcd_machine = o3d.io.read_point_cloud(path_machine)
        self.mesh_machine = o3d.io.read_triangle_mesh(path_mesh)

        self.relation = self.data_manager.load_var_pickle(relation_path)

        #  Robot
        self.arm = Doosan()

        # Global var for actual guard
        self.all_poses_check = False
        self.actual_index_pose = 0
        # All the points to check for the actuel guard pcd
        self.potential_guards = o3d.geometry.PointCloud()
        # Transform tf machine to tf world (base_0) for actual spot
        self.trans_base = TransformStamped()

        # Dictionnary associate to the mobile base spot
        self.dict_pos2pts = {}
        self.offline_trajectory = {}

        # The joint's configuration to save
        self.j1 = 0
        self.j2 = 0
        self.j3 = 0
        self.j4 = 0
        self.j5 = 0
        self.j6 = 0

        # TF broadcaster (need to be the same variable to change the same tf)
        self.broadcaster_cone = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster_guard = tf2_ros.StaticTransformBroadcaster()

        # TF reader
        self.tf_buffer = tf2_ros.Buffer()
        # Start the TF listen and store to buffer (10s buffer)
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscriber
        # self.pcd_sub = rospy.Subscriber("/pcl_result", PointCloud2, self.callback_fov)

        # Publisher
        # self.pcl_pub = rospy.Publisher('/transf_mat', Float64MultiArray, queue_size=10)

    def select_best_spot(self, relation, pcd_initial, limit=10, index_it=0):
        extracted_values = {}
        max_value = 0
        first_iteration = True
        while relation:
            longest_key = max(relation, key=lambda key: len(relation[key]))

            longest_value = relation[longest_key]
            # longest_value = [tuple(i) for i in longest_value]

            if first_iteration:
                max_value = len(longest_value)
                first_iteration = False

            if not np.any(longest_value) or len(longest_value) <= int(max_value / limit):
                # rospy.loginfo("Dict Stop")
                # rospy.loginfo(len(longest_value))
                # rospy.loginfo(max_value)
                break

            if np.shape(longest_key)[0] == 2:
                new_longest_key = longest_key + (1,)
            else:
                new_longest_key = longest_key

            extracted_values[new_longest_key] = longest_value
            del relation[longest_key]

            with alive_bar(len(relation.items())) as bar:
                for key, value in relation.items():
                    relation[key] = [point for point in value if point not in longest_value]
                    bar()
        self.data_manager.save_var_pickle(extracted_values,
                                          f"/home/will/PycharmProjects/Bertrand/data/extracted_values_{index_it}.pkl")
        for key, value in extracted_values.items():
            color = [random.uniform(0.0, 1.0), random.uniform(0.0, 1.0), random.uniform(0.0, 1.0)]
            pcd = self.open3d_tool.np2pcd(np.array(value).reshape(len(value), 3))
            pcd.paint_uniform_color(color)
            extracted_values[key] = pcd

        for key, value in extracted_values.items():
            dists = pcd_initial.compute_point_cloud_distance(value)
            dists = np.asarray(dists)
            ind = np.where(dists < 0.001)[0]
            extracted_values[key] = pcd_initial.select_by_index(ind)
        # rospy.loginfo("exit")

        return extracted_values

    def select_best_guard(self, relation, pcd_initial, limit=10, index_it=0):
        extracted_values = {}
        max_value = 0
        first_iteration = True
        while relation:
            # create the Indice dictionnary
            relation_indice = {}
            for elem in relation.keys():
                I = round(math.pow(self.arm.get_manipulability(list(elem)) *
                                   self.arm.get_joint_limit_index(list(elem)) / self.arm.joint_index_max, 1) *
                          math.pow(len(relation[elem]) / len(self.pcd_machine.points), 0.5), 4)
                # print(I)
                relation_indice[I] = elem

            best_key = max(relation_indice.keys())
            best_config = relation_indice[best_key]
            longest_value = relation[best_config]

            if first_iteration:
                max_value = len(longest_value)
                first_iteration = False

            if not np.any(longest_value) or len(longest_value) <= int(max_value / limit):
                # rospy.loginfo("Dict Stop")
                # rospy.loginfo(len(longest_value))
                # rospy.loginfo(max_value)
                break

            extracted_values[best_config] = longest_value
            del relation[best_config]

            with alive_bar(len(relation.items())) as bar:
                for key, value in relation.items():
                    relation[key] = [point for point in value if point not in longest_value]
                    bar()
        self.data_manager.save_var_pickle(extracted_values,
                                          f"/home/will/PycharmProjects/Bertrand/data/extracted_values_{index_it}.pkl")
        for key, value in extracted_values.items():
            color = [random.uniform(0.0, 1.0), random.uniform(0.0, 1.0), random.uniform(0.0, 1.0)]
            pcd = self.open3d_tool.np2pcd(np.array(value).reshape(len(value), 3))
            pcd.paint_uniform_color(color)
            extracted_values[key] = pcd

        for key, value in extracted_values.items():
            dists = pcd_initial.compute_point_cloud_distance(value)
            dists = np.asarray(dists)
            ind = np.where(dists < 0.001)[0]
            extracted_values[key] = pcd_initial.select_by_index(ind)
        # rospy.loginfo("exit")

        return extracted_values

    def save_visible_points(self, msg):
        """
        Receive the pcd seen by the emulated camera
        :param msg: pcd
        :return:
        """
        if self.all_poses_check:
            return
        # Add points to dict
        pcd = o3d_ros.rospc_to_o3dpc(msg)
        self.dict_pos2pts[(self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)] = [tuple(i) for i in
                                                                                     np.trunc(np.around(
                                                                                         np.asarray(pcd.points),
                                                                                         decimals=5) * 10 ** 4) / (
                                                                                             10 ** 4)]

    def check_guard_validity(self, guard):
        """

        :param guard:poseStamped
        :return:
        """

        # Get inverse kinematics
        ikin = self.arm.ikin_moveit(guard.pose, "camera_base")

        if ikin.error_code.val == 1:
            tcp_map = guard  # self.arm.fkin_moveit(pose, "camera_base").pose_stamped[0]

            quaternion = (
                tcp_map.pose.orientation.x,
                tcp_map.pose.orientation.y,
                tcp_map.pose.orientation.z,
                tcp_map.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)

            # check = self.arm.check_collision(pose)
            joint_configuration = ikin.solution.joint_state.position
            # if check.valid:
            plan_valid = self.arm.is_plan_valid(joint_configuration)
            if plan_valid:
                self.j1 = joint_configuration[0]
                self.j2 = joint_configuration[1]
                self.j3 = joint_configuration[2]
                self.j4 = joint_configuration[3]
                self.j5 = joint_configuration[4]
                self.j6 = joint_configuration[5]
                return True, ikin, [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]
        return False, [], []

    def send_next_valid_config(self):
        """
        Send a valid joint configuration to the emulate camera (cone)
        :return:
        """
        # if self.all_poses_check:
        #     return

        tcp_machine = PoseStamped()
        is_valid = False
        while not is_valid:

            # Get next pose
            poses, self.all_poses_check = self.get_next_pose()
            # Check the guard pose (collision, ikin...) and get inverse kinematics (none if invalide)
            is_valid, ikin, joint_configuration = self.check_guard_validity(poses)

            if self.all_poses_check:
                print("All poses check for this spot")
                return

        matrix, tcp_machine = self.get_fov_transformation(poses)

        # send tf
        static_transform_stamped = TransformStamped()
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

        self.broadcaster_cone.sendTransform(static_transform_stamped)

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
        return self.fov_emulator.get_visible_point(msg)

    def get_fov_transformation(self, pose):
        '''
       Return the TCP pose in the map frame
       :param pose: The tcp pose in the map
       :return: translatio matrix  and is pose valide
       '''

        # transform the pose from the map to the base_footprint
        try:
            trans_machine = self.tf_buffer.lookup_transform('machine', 'world', rospy.Time(),
                                                            rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")
        tcp_machine = tf2_geometry_msgs.do_transform_pose(pose, trans_machine)
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
        display_marker(Marker.ARROW,
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

    def get_next_pose(self):
        """
        Get the position and orientation of the next potential guard
        :return: pose and bool (if all points from the pcd have been checked)
        """
        all_poses_check = False
        # get the new pose
        position = self.potential_guards.points[self.actual_index_pose]

        # Get the inverse unit vector normal (we want to look at the plane not from it)
        quaternion = self.get_guard_orient_from_vector(self.potential_guards.normals[self.actual_index_pose])

        pose = PoseStamped()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        # increase actual index pose to check
        self.actual_index_pose += 1

        # if all poses check return true
        if self.actual_index_pose >= len(np.asarray(self.potential_guards.points)):
            all_poses_check = True
            # rospy.loginfo("Last point for this spot")
            self.actual_index_pose = 0

        # rospy.loginfo("Point nb: %d / %d", self.actual_index_pose, len(np.asarray(self.potential_guards.points)))

        # We had the point in the machine frame also we transform it in the robot base frame
        pose = tf2_geometry_msgs.do_transform_pose(pose, self.trans_base)

        # send tf
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = "world"
        static_transform_stamped.child_frame_id = "guard"
        static_transform_stamped.transform.translation.x = pose.pose.position.x
        static_transform_stamped.transform.translation.y = pose.pose.position.y
        static_transform_stamped.transform.translation.z = pose.pose.position.z
        static_transform_stamped.transform.rotation.x = pose.pose.orientation.x
        static_transform_stamped.transform.rotation.y = pose.pose.orientation.y
        static_transform_stamped.transform.rotation.z = pose.pose.orientation.z
        static_transform_stamped.transform.rotation.w = pose.pose.orientation.w

        self.broadcaster_guard.sendTransform(static_transform_stamped)

        return pose, all_poses_check

    def get_guard_orient_from_vector(self, guard_vector):
        vector_dir = - guard_vector
        vector_dir = vector_dir / np.linalg.norm(vector_dir)

        # Get guard orientation
        vector_projet = np.array([vector_dir[0], vector_dir[1], 0])  # On projet le vecteur sur le plan XY
        vector_projet = vector_projet / np.linalg.norm(vector_projet)
        rot_z = R.from_euler("z", 90, degrees=True)
        vector_trans_z = np.dot(rot_z.as_matrix(), vector_projet)

        vector_dir_norm = vector_dir / np.linalg.norm(vector_dir)
        w = np.cross(vector_dir_norm, vector_trans_z)
        rot = np.column_stack((vector_dir_norm, vector_trans_z, w))
        r = R.from_matrix(rot)
        r_quat = r.as_quat()
        return r_quat

    def get_transform_base(self):
        """
        get tha transform matrix between machine and arm base (world)
        :return: update in global variable
        """
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            self.trans_base = tf_buffer.lookup_transform('world', 'machine', rospy.Time(),
                                                         rospy.Duration(10.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")
        # rospy.loginfo(self.trans_base)

    def generate_pcd_guard(self, pcd_init, dist=0.4, pnt_percent=10):
        """
        get pcd and generate guards with the same normal
        :param pcd_init:
        :return:
        """
        # Set the distance of 10 cm
        distance = dist

        # Initialize arrays to store the generated points and corresponding normals
        points_sampled = []
        normals_sampled = []

        # Generate a number of random points equal to the number of points in the initial point cloud
        n_points = int(len(pcd_init.points) / pnt_percent)
        if n_points <= 400:
            n_points = 400
        if n_points >= len(pcd_init.points):
            n_points = len(pcd_init.points)
        for index in range(n_points):
            # Select a random point in the initial point cloud
            index_rnd = np.random.randint(len(pcd_init.points))
            point_rnd = pcd_init.points[index_rnd]
            normal_rnd = pcd_init.normals[index_rnd]

            # Generate a new point by placing it along the normal at a distance of 10 cm from the initial point
            point_sampled = point_rnd + (distance * normal_rnd)
            points_sampled.append(point_sampled)
            normals_sampled.append(normal_rnd)

        # Create a new point cloud from the generated points and normals
        pcd_sampled = o3d.geometry.PointCloud()
        pcd_sampled.points = o3d.utility.Vector3dVector(points_sampled)
        pcd_sampled.normals = o3d.utility.Vector3dVector(normals_sampled)
        # o3d.visualization.draw_geometries([pcd_sampled])
        # Visualize the new point cloud
        # pcd_sampled.uniform_down_sample(100)
        # o3d.visualization.draw_geometries([pcd_sampled])
        return pcd_sampled

    def cluster_pcd(self, pcd, esp=0.5, min_points=10):
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                pcd.cluster_dbscan(eps=esp, min_points=min_points, print_progress=True))

        max_label = labels.max()
        # print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        # o3d.visualization.draw_geometries([pcd])
        cluster_centers = []
        clusters_pcds = []
        # get pcds split
        for i in range(max_label):
            ind = np.where(labels == i)[0]
            cluster = pcd.select_by_index(ind)
            clusters_pcds.append(cluster)
            cluster_center = cluster.get_center()
            cluster_centers.append(cluster_center)
        return cluster_centers, clusters_pcds

    def chose_spot_from_vectors(self, vectors, spots, cluster_center):
        # Choix spot associate

        angles = np.arctan2(vectors[:, 1], vectors[:, 0])
        means_angle = np.mean(angles)
        spots_angles = np.arctan2((spots[:, 1] - cluster_center[1]), (spots[:, 0] - cluster_center[0]))

        diff_angles = abs(means_angle - spots_angles)

        cluster_spot = min(diff_angles)
        index_spot = np.where(diff_angles == cluster_spot)[0]
        spot_associate = spots[index_spot]

        # spot_associate[0][2] = 1  # set z to 0 (2d position to 3d) for visualisation

        # spot_associate_pcd = self..np2pcd(spot_associate).paint_uniform_color([0, 0, 1])
        return spot_associate[0]


if __name__ == '__main__':
    rospy.init_node('offline_strategie', anonymous=True)
    debug = rospy.get_param("/debug", default=False)
    pcd_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_pcd")
    path_relation = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/relation_spot")
    mesh_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_ply")
    offline = Offline(pcd_path, mesh_path, path_relation)

    # answer = input("Do you want to generate the mobile base spots ? (around 1H) y/n: ")
    # if answer == "y" or answer == "yes":
    if rospy.get_param("/compute_best_spots"):
        # Get the best spots for the MiR
        best_spots = offline.select_best_spot(offline.relation, offline.pcd_machine,
                                              limit=rospy.get_param("/spot_limit"))
        # print(list(best_spots.keys()))
        offline.data_manager.save_var_pickle(list(best_spots.keys()),
                                             rospkg.RosPack().get_path('cleaning') + rospy.get_param("/spots"))
        offline.data_manager.save_pcd_list(list(best_spots.values()),
                                           rospkg.RosPack().get_path('cleaning') + rospy.get_param("/spots_pcds"))
    spots = []
    spots = offline.data_manager.load_var_pickle(
        rospkg.RosPack().get_path('cleaning') + rospy.get_param("/spots"))
    print(spots)
    pcds_spots = offline.data_manager.load_pcd_list(rospkg.RosPack().get_path('cleaning') +
                                                    rospy.get_param("/spots_pcds"))
    # print(pcds_spots)
    # answer = input("Do you want to generate the robot's guards ? y/n: ")
    # if answer == "y" or answer == "yes" or rospy.get_param("/compute_relation_guards"):
    if rospy.get_param("/compute_relation_guards"):
        best_spots = {}
        for index, pcd in enumerate(pcds_spots):
            best_spots[spots[index]] = pcd
        #  Change the TCP for the compute
        offline.arm.set_tcp("camera_base")

        index = 0
        for spot, pcd in best_spots.items():

            offline.all_poses_check = False

            # print(spot[0], spot[1], spot[2], offline.robot.mir_pose_angle(spot, pcd))

            offline.robot.move_mobile_base(spot[0], spot[1], offline.robot.mir_pose_angle(spot, pcd))
            # rospy.sleep(10)
            collision_stamp = PoseStamped()
            collision_stamp.header.frame_id = "machine"
            collision_stamp.pose.orientation.w = 1
            offline.arm.add_machine_colision(rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_mesh"),
                                             collision_stamp)
            # rospy.sleep(10)

            # compute transform machine to base
            offline.get_transform_base()

            # Set potential guards
            offline.potential_guards = offline.generate_pcd_guard(pcd_init=pcd,
                                                                  pnt_percent=rospy.get_param("/potential_guard"))

            # Get all guards
            # continue While all poses check
            while not offline.all_poses_check:
                # offline.get_next_pose()
                offline.save_visible_points(offline.send_next_valid_config())

            file_name = rospy.get_param("/relation_guards") + f"{index}.pkl"

            offline.data_manager.save_var_pickle(offline.dict_pos2pts,
                                                 rospkg.RosPack().get_path('cleaning') + file_name)
            # print(offline.dict_pos2pts)
            offline.dict_pos2pts.clear()
            index += 1

    # save global results
    # answer = input("Do you to recalculate the guards ? y/n: ")
    # if answer == "y" or answer == "yes":
    resultat = []
    resultat = np.asarray(resultat)
    spots = np.asarray(spots)
    observed_pcd = []
    observed_pcd = np.asarray(observed_pcd)
    if (debug):
        o3d.visualization.draw_geometries(pcds_spots)
    for index, spot in enumerate(spots):
        print(spot)
        spot[2] = offline.robot.mir_pose_angle(spot, pcds_spots[index])
        offline.robot.move_mobile_base(spot[0], spot[1], spot[2])
        # rospy.sleep(10)
        collision_stamp = PoseStamped()
        collision_stamp.header.frame_id = "machine"
        collision_stamp.pose.orientation.w = 1
        offline.arm.add_machine_colision(rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_mesh"),
                                         collision_stamp)
        # rospy.sleep(10)

        file_name = rospy.get_param("/relation_guards") + f"{index}.pkl"
        offline.dict_pos2pts = offline.data_manager.load_var_pickle(rospkg.RosPack().get_path('cleaning') + file_name)

        # Select best guards
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            trans_machine = tf_buffer.lookup_transform('machine', 'world', rospy.Time(),
                                                       rospy.Duration(10.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")

        result_dict = offline.select_best_guard(offline.dict_pos2pts, offline.pcd_machine,
                                                limit=rospy.get_param("/guards_limit"), index_it=index)
        best_guard_configurations = list(map(list, list(result_dict.keys())))

        guard_observation_pcd = list(result_dict.values())
        observed_pcd = np.concatenate((observed_pcd, guard_observation_pcd), axis=0)
        guard = []
        for configuration in best_guard_configurations:
            fkin = offline.arm.fkin_moveit(configuration, "camera_base")
            fkin = tf2_geometry_msgs.do_transform_pose(fkin.pose_stamped[0], trans_machine)
            pose = np.array([fkin.pose.position.x, fkin.pose.position.y, fkin.pose.position.z])
            guard.append(pose)

        guard = offline.open3d_tool.np2pcd(guard)
        guard.paint_uniform_color([1, 0, 0])
        # guard_spots_pcd = np.append(guard_spots_pcd, guard)

        guard_np = np.asarray(guard.points)

        if np.size(best_guard_configurations, 0) > 1:
            # get distance matrix in radian in miliseconde
            dist_matrix = ((offline.robot.joint_tsp(np.asarray(best_guard_configurations),
                                                    offline.arm.get_joint_velocity_limit())) * 1000).astype(int)
            # dist_matrix = offline.open3d_tool.check_obs_btw_points(dist_matrix, guard_np, offline.mesh_machine)
            # Generate path
            permutation = offline.robot.generate_trajectory(dist_matrix)

            # Update the final trajectory
            offline.offline_trajectory[tuple(spot)] = offline.robot.sort_poses(best_guard_configurations, permutation)
            guard_np = offline.robot.sort_poses(guard_np, permutation)
            # DEBUG
            if debug:
                lines = []
                for i in range(len(permutation)):
                    lines.append([permutation[i], permutation[(i + 1) % len(permutation)]])

                colors = [[1, 0, 0] for i in range(len(lines))]
                line_set = o3d.geometry.LineSet()
                line_set.points = o3d.utility.Vector3dVector(guard_np)
                line_set.lines = o3d.utility.Vector2iVector(lines)
                line_set.colors = o3d.utility.Vector3dVector(colors)

                o3d.visualization.draw_geometries([guard, offline.pcd_machine, line_set])
        else:
            offline.offline_trajectory[tuple(spot)] = []
    file = rospy.get_param("/guards_pcd")
    o3d.io.write_point_cloud(rospkg.RosPack().get_path('cleaning') + file + ".ply", guard)
    file = rospy.get_param("/observed_pcd")
    offline.data_manager.save_pcd_list(observed_pcd, rospkg.RosPack().get_path('cleaning') + file)

    # NEW PART

    file = rospy.get_param("/observed_pcd")
    pcds = offline.data_manager.load_pcd_list(rospkg.RosPack().get_path('cleaning') + file)
    # file_name = rospy.get_param("/trajectorie")
    # offline.offline_trajectory = offline.data_manager.load_var_pickle(rospkg.RosPack().get_path('cleaning') + file_name)
    # Generate non obsevable area
    observable_pcd = offline.open3d_tool.fuse_pcds(pcds)
    # Search for non observed point
    non_obs_pcd = offline.open3d_tool.compare_pcd(offline.pcd_machine, observable_pcd, "sup")

    # Get center from each cluster non observable
    cluster_center_np, clusters_pcds = offline.cluster_pcd(non_obs_pcd, esp=0.02)

    # Ray Cast
    # Genreate ray around machine
    scene = o3d.t.geometry.RaycastingScene()
    machine_mesh = o3d.t.geometry.TriangleMesh.from_legacy(offline.mesh_machine)
    cube_id = scene.add_triangles(machine_mesh)
    # Add ground
    ground = o3d.geometry.TriangleMesh.create_box(width=1, height=1.5, depth=0.001)
    ground.translate((0, -1.5, 0), relative=True)
    ground = o3d.t.geometry.TriangleMesh.from_legacy(ground)
    ground_id = scene.add_triangles(ground)

    center = offline.mesh_machine.get_center()
    max_bound = offline.mesh_machine.get_max_bound()

    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=np.linalg.norm(max_bound), resolution=20)
    mesh_sphere.translate(center)
    pcd_ray_dest = mesh_sphere.sample_points_poisson_disk(number_of_points=1000)

    # generate raycast for cluster centers
    cluster_np = np.asarray(cluster_center_np)
    dest_np = np.asarray(pcd_ray_dest.points)
    print(cluster_np)
    observable_concavity = []
    # For each cluster
    for i, clust_pt in enumerate(cluster_np):
        rays, line, line_color, line_indice = offline.open3d_tool.generate_rays_vectors(clust_pt, dest_np)
        ans = scene.cast_rays(rays)
        # get rays not in collision
        rays_hit = ans['t_hit'].numpy()

        indice = np.where(rays_hit == float('inf'))[0]
        non_obs_cluster = np.where(clusters_pcds[i])

        # get all rays that can get out
        indice = np.where(rays_hit != float('inf'))[0]
        rays_out = np.delete(rays.numpy(), indice, axis=0)

        # Remove rays origines to keep only vector of each rays
        vectors_out = rays_out[:, 3:]

        # Select spot associate
        if vectors_out.size != 0:
            clusterise = offline.open3d_tool.clusterise_dbscan(vectors_out)
            print(clusterise)
            if clusterise:
                # Get best vector
                biggest_exit = max(clusterise, key=len)
                best_vec_exit = vectors_out[biggest_exit]
                rospy.loginfo("Best spot:")

                spots = np.asarray(list(map(list, list(offline.offline_trajectory.keys()))))
                # rospy.loginfo(spots)
                spot = offline.chose_spot_from_vectors(best_vec_exit, spots, clust_pt)
                rospy.loginfo(spot)
                # Choix orientation et position TCP
                # trans to spherical
                rayons, thetas, phis = offline.open3d_tool.euler2polar(best_vec_exit[:, 0], best_vec_exit[:, 1],
                                                                       best_vec_exit[:, 2])

                # TODO, Here change to sample from 0.3 to 1m by 10cm until the pos is valid
                means_thetas = np.mean(thetas)
                means_phis = np.mean(phis)
                r = 0.4

                # Setup the scene
                offline.all_poses_check = False

                offline.arm.set_tcp("camera_base")
                # print(spot[0], spot[1], spot[2], offline.robot.mir_pose_angle(spot, pcd))

                offline.robot.move_mobile_base(spot[0], spot[1], spot[2])
                # rospy.sleep(10)
                collision_stamp = PoseStamped()
                collision_stamp.header.frame_id = "machine"
                collision_stamp.pose.orientation.w = 1
                offline.arm.add_machine_colision(
                    rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_mesh"),
                    collision_stamp)
                # rospy.sleep(10)

                # compute transform machine to base
                offline.get_transform_base()
                print("Test all 7 guards")
                for r in np.arange(0.1, 0.7, 0.1):
                    print("In the for loop")
                    # coord for guard:
                    vector_guard = offline.open3d_tool.polar2euler(means_thetas, means_phis, r)

                    guard_orientation = offline.get_guard_orient_from_vector(vector_guard)
                    guard_position = np.asarray(offline.open3d_tool.np2pcd([vector_guard]).translate(clust_pt).points)
                    # rospy.loginfo(guard_orientation)
                    # rospy.loginfo(guard_position)
                    # test new guard
                    guard = PoseStamped()
                    guard.header.frame_id = "machine"
                    guard.pose.position.x = guard_position[0][0]
                    guard.pose.position.y = guard_position[0][1]
                    guard.pose.position.z = guard_position[0][2]
                    guard.pose.orientation.x = guard_orientation[0]
                    guard.pose.orientation.y = guard_orientation[1]
                    guard.pose.orientation.z = guard_orientation[2]
                    guard.pose.orientation.w = guard_orientation[3]

                    # send tf
                    static_transform_stamped = TransformStamped()
                    static_transform_stamped.header.stamp = rospy.Time.now()
                    static_transform_stamped.header.frame_id = "machine"
                    static_transform_stamped.child_frame_id = "guard"
                    static_transform_stamped.transform.translation.x = guard.pose.position.x
                    static_transform_stamped.transform.translation.y = guard.pose.position.y
                    static_transform_stamped.transform.translation.z = guard.pose.position.z
                    static_transform_stamped.transform.rotation.x = guard.pose.orientation.x
                    static_transform_stamped.transform.rotation.y = guard.pose.orientation.y
                    static_transform_stamped.transform.rotation.z = guard.pose.orientation.z
                    static_transform_stamped.transform.rotation.w = guard.pose.orientation.w

                    offline.broadcaster_guard.sendTransform(static_transform_stamped)

                    # transform to world frame
                    guard = tf2_geometry_msgs.do_transform_pose(guard, offline.trans_base)

                    is_valid, ikin, joint_configuration = offline.check_guard_validity(guard)

                    if is_valid:
                        print("Find a valid pose")
                        matrix, tcp_machine = offline.get_fov_transformation(guard)

                        # send tf
                        static_transform_stamped = TransformStamped()
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

                        offline.broadcaster_cone.sendTransform(static_transform_stamped)

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
                        pcd_observable = offline.fov_emulator.get_visible_point(msg)
                        observable_concavity.append(o3d_ros.rospc_to_o3dpc(pcd_observable))
                        # save pts at spot index
                        # print(type(joint_configuration))
                        value = np.asarray(offline.offline_trajectory.get(tuple(spot)))
                        if not np.any(value):
                            value = []
                        # print(value)
                        else:
                            value = value.tolist()
                        print(type(value))
                        new_joints = value.append(joint_configuration)
                        offline.offline_trajectory.update({tuple(spot): new_joints})
                        # rospy.loginfo("resultats:")
                        # rospy.loginfo(offline.offline_trajectory[tuple(spot)])
                        # rospy.loginfo(joint_configuration)

                        # np.append(offline.offline_trajectory[tuple(spot)], [joint_configuration], axis=0)
                        break

    # rospy.loginfo(observable_concavity)
    # save new observables surfaces
    file = rospy.get_param("/observable_concavity")
    offline.data_manager.save_pcd_list(observable_concavity, rospkg.RosPack().get_path('cleaning') + file)

    # input("finish !!")

    # END NEW PART
    # set spot z to 1m, only for better visual understanding
    spots[:, 2] = 1

    spots = offline.open3d_tool.np2pcd(spots)
    spots.paint_uniform_color([0, 1, 0])
    resultat = np.append(resultat, observed_pcd)
    resultat = np.append(resultat, guard)
    resultat = np.append(resultat, spots)
    # file = rospy.get_param("/result_pcd")
    # offline.data_manager.save_pcd_list(resultat, rospkg.RosPack().get_path('cleaning') + file)
    # o3d.visualization.draw_geometries(resultat)

    #  Change the TCP at the end of the compute
    offline.arm.set_tcp("tcp")

    file_name = rospy.get_param("/trajectorie")
    offline.data_manager.save_var_pickle(offline.offline_trajectory, rospkg.RosPack().get_path('cleaning') + file_name)
    print("Offline strategie done")
