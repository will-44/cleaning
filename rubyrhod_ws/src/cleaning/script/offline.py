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
from geometry_msgs.msg import PoseStamped, TransformStamped
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


class Offline:

    def __init__(self, path_machine, relation_path, debug=False):

        self.data_manager = DataManager()
        self.open3d_tool = Open3dTool()
        self.robot = Robot()

        self.debug = debug

        # PCD and Mesh of the machine
        self.pcd_machine = o3d.io.read_point_cloud(path_machine)
        self.mesh_machine = o3d.io.read_triangle_mesh(path_machine)

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
        self.pcd_sub = rospy.Subscriber("/pcl_result", PointCloud2, self.callback_fov)

        # Publisher
        self.pcl_pub = rospy.Publisher('/transf_mat', Float64MultiArray, queue_size=10)

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
                rospy.loginfo("Dict Stop")
                rospy.loginfo(len(longest_value))
                rospy.loginfo(max_value)
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
        rospy.loginfo("exit")

        return extracted_values

    def select_best_guard(self, relation, pcd_initial, limit=10, index_it=0):
        extracted_values = {}
        max_value = 0
        first_iteration = True
        while relation:
            # create the Indice dictionnary
            relation_indice = {}
            for elem in relation.keys():
                I = round(self.arm.get_manipulability(list(elem)) * self.arm.get_joint_limit_index(list(elem)) * len(relation[elem]), 4)
                relation_indice[I] = elem

            best_key = max(relation_indice.keys())
            best_config = relation_indice[best_key]
            longest_value = relation[best_config]

            if first_iteration:
                max_value = len(longest_value)
                first_iteration = False

            if not np.any(longest_value) or len(longest_value) <= int(max_value / limit):
                rospy.loginfo("Dict Stop")
                rospy.loginfo(len(longest_value))
                rospy.loginfo(max_value)
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
        rospy.loginfo("exit")

        return extracted_values

    def callback_fov(self, msg):
        """
        Receive the pcd seen by the emulated camera
        :param msg: pcd
        :return:
        """
        # Add points to dict
        pcd = o3d_ros.rospc_to_o3dpc(msg)
        self.dict_pos2pts[(self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)] = [tuple(i) for i in
                                                                                     np.trunc(np.around(
                                                                                         np.asarray(pcd.points),
                                                                                         decimals=5) * 10 ** 4) / (
                                                                                             10 ** 4)]

        self.send_next_valid_config()

    def send_next_valid_config(self):
        """
        Send a valid joint configuration to the emulate camera (cone)
        :return:
        """
        if self.all_poses_check:
            return

        tcp_machine = PoseStamped()
        is_valid = False
        while not is_valid:

            # Get next pose
            poses, self.all_poses_check = self.get_next_pose()

            # Get inverse kinematics
            ikin = self.arm.ikin_moveit(poses, "camera_base")

            if ikin.error_code.val == 1:
                matrix, is_valid, tcp_machine = self.get_fov_transformation(ikin.solution.joint_state.position)
            if self.all_poses_check:
                print("All poses check for this spot")
                return

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
        self.pcl_pub.publish(msg)

    def get_fov_transformation(self, pose):
        '''
       Return the TCP pose in the map frame
       :param pose: joint pose
       :return: translatio matrix  and is pose valide
       '''

        tcp_map = self.arm.fkin_moveit(pose, "camera_base").pose_stamped[0]

        quaternion = (
            tcp_map.pose.orientation.x,
            tcp_map.pose.orientation.y,
            tcp_map.pose.orientation.z,
            tcp_map.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        check = self.arm.check_collision(pose)

        if check.valid:
            plan_valid = self.arm.is_plan_valid(pose)
            if plan_valid:
                self.j1 = pose[0]
                self.j2 = pose[1]
                self.j3 = pose[2]
                self.j4 = pose[3]
                self.j5 = pose[4]
                self.j6 = pose[5]

                # transform the pose from the map to the base_footprint
                try:
                    trans_machine = self.tf_buffer.lookup_transform('machine', 'world', rospy.Time(),
                                                                    rospy.Duration(1))
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

                return cone_transform[0], True, tcp_machine
        return [0, 0, 0], False, [0, 0, 0]

    def get_next_pose(self):
        """
        Get the position and orientation of the next potential guard
        :return: pose and bool (if all points from the pcd have been checked)
        """
        all_poses_check = False
        # get the new pose
        position = self.potential_guards.points[self.actual_index_pose]

        # Get the inverse unit vector normal (we want to look at the plane not from it)
        vector_dir = - self.potential_guards.normals[self.actual_index_pose]

        # Get guard orientation
        vector_projet = np.array([vector_dir[0], vector_dir[1], 0])  # On projet le vecteur sur le plan XY
        vector_projet = vector_projet / np.linalg.norm(vector_projet)
        rot_z = R.from_euler("z", 90, degrees=True)
        vector_trans_z = np.dot(rot_z.as_matrix(), vector_projet)

        vector_dir_norm = vector_dir / np.linalg.norm(vector_dir)
        w = np.cross(vector_dir_norm, vector_trans_z)
        rot = np.column_stack((vector_dir_norm, vector_trans_z, w))
        r = R.from_matrix(rot)

        # Transform to quaternion
        quaternion = r.as_quat()
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
            rospy.loginfo("Last point for this spot")
            self.actual_index_pose = 0

        rospy.loginfo("Point nb: %d / %d", self.actual_index_pose, len(np.asarray(self.potential_guards.points)))

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

        return pose.pose, all_poses_check

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
        rospy.loginfo(self.trans_base)

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


if __name__ == '__main__':
    rospy.init_node('offline_strategie', anonymous=True)
    debug = rospy.get_param("/debug", default=False)
    pcd_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_pcd")
    path_relation = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/relation_spot")

    offline = Offline(pcd_path, path_relation)

    answer = input("Do you want to generate the mobile base spots ? (around 1H) y/n: ")
    if answer == "y" or answer == "yes":
        # Get the best spots for the MiR
        best_spots = offline.select_best_spot(offline.relation, offline.pcd_machine,
                                              limit=rospy.get_param("/spot_limit"))
        print(list(best_spots.keys()))
        offline.data_manager.save_var_pickle(list(best_spots.keys()),
                                             rospkg.RosPack().get_path('cleaning') + rospy.get_param("/spots"))
        offline.data_manager.save_pcd_list(list(best_spots.values()),
                                           rospkg.RosPack().get_path('cleaning') + rospy.get_param("/spots_pcds"))
    spots = []
    spots = offline.data_manager.load_var_pickle(
        rospkg.RosPack().get_path('cleaning') + rospy.get_param("/spots"))

    pcds_spots = offline.data_manager.load_pcd_list(rospkg.RosPack().get_path('cleaning') +
                                                    rospy.get_param("/spots_pcds"))
    print(pcds_spots)
    answer = input("Do you want to generate the robot's guards ? y/n: ")
    if answer == "y" or answer == "yes":
        best_spots = {}
        for index, pcd in enumerate(pcds_spots):
            best_spots[spots[index]] = pcd
        #  Change the TCP for the compute
        offline.arm.set_tcp("camera_base")

        index = 0
        for spot, pcd in best_spots.items():

            offline.all_poses_check = False

            print(spot[0], spot[1], spot[2], offline.robot.mir_pose_angle(spot, pcd))

            offline.robot.move_mobile_base(spot[0], spot[1], offline.robot.mir_pose_angle(spot, pcd))
            rospy.sleep(10)
            collision_stamp = PoseStamped()
            collision_stamp.header.frame_id = "machine"
            collision_stamp.pose.orientation.w = 1
            offline.arm.add_machine_colision(rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_mesh"),
                                             collision_stamp)
            rospy.sleep(10)

            # compute transform machine to base
            offline.get_transform_base()

            # Set potential guards
            offline.potential_guards = offline.generate_pcd_guard(pcd_init=pcd,
                                                                  pnt_percent=rospy.get_param("/potential_guard"))

            # Get all guards
            offline.send_next_valid_config()

            # While all poses check sleep
            while not offline.all_poses_check:
                rospy.sleep(1)

            file_name = rospy.get_param("/relation_guards") + f"{index}.pkl"

            offline.data_manager.save_var_pickle(offline.dict_pos2pts,
                                                 rospkg.RosPack().get_path('cleaning') + file_name)
            print(offline.dict_pos2pts)
            offline.dict_pos2pts.clear()
            index += 1

    resultat = []
    resultat = np.asarray(resultat)
    spots = np.asarray(spots)
    observed_pcd = []
    observed_pcd = np.asarray(observed_pcd)
    if (debug):
        o3d.visualization.draw_geometries(pcds_spots)
    for index, spot in enumerate(spots):
        spot[2] = offline.robot.mir_pose_angle(spot, pcds_spots[index])
        offline.robot.move_mobile_base(spot[0], spot[1], spot[2])
        # rospy.sleep(10)
        collision_stamp = PoseStamped()
        collision_stamp.header.frame_id = "machine"
        collision_stamp.pose.orientation.w = 1
        offline.arm.add_machine_colision(rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_mesh"),
                                         collision_stamp)
        rospy.sleep(10)

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
        best_guard = list(result_dict.keys())
        guard_observation_pcd = list(result_dict.values())
        observed_pcd = np.concatenate((observed_pcd, guard_observation_pcd), axis=0)
        guard = []
        for configuration in best_guard:
            fkin = offline.arm.fkin_moveit(configuration, "camera_base")
            fkin = tf2_geometry_msgs.do_transform_pose(fkin.pose_stamped[0], trans_machine)
            pose = np.array([fkin.pose.position.x, fkin.pose.position.y, fkin.pose.position.z])
            guard.append(pose)

        guard = offline.open3d_tool.np2pcd(guard)
        guard.paint_uniform_color([1, 0, 0])
        # guard_spots_pcd = np.append(guard_spots_pcd, guard)

        guard_np = np.asarray(guard.points)

        if np.size(guard_np, 0) > 1:
            # Check guards accessibility
            dist_matrix = ((euclidean_distance_matrix(guard_np)) * 1000).astype(int)
            dist_matrix = offline.open3d_tool.check_obs_btw_points(dist_matrix, guard_np, offline.mesh_machine)
            # Generate path
            permutation = offline.robot.generate_trajectory(dist_matrix)

            # Update the final trajectory
            offline.offline_trajectory[tuple(spot)] = offline.robot.sort_poses(best_guard, permutation)

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

    file = rospy.get_param("/guards_pcd")
    o3d.io.write_point_cloud(rospkg.RosPack().get_path('cleaning') + file + ".ply", guard)
    file = rospy.get_param("/observed_pcd")
    offline.data_manager.save_pcd_list(observed_pcd, rospkg.RosPack().get_path('cleaning') + file)

    # set spot z to 1m, only for better visual understanding
    spots[:, 2] = 1

    spots = offline.open3d_tool.np2pcd(spots)
    spots.paint_uniform_color([0, 1, 0])
    resultat = np.append(resultat, observed_pcd)
    resultat = np.append(resultat, guard)
    resultat = np.append(resultat, spots)
    file = rospy.get_param("/result_pcd")
    offline.data_manager.save_pcd_list(resultat, rospkg.RosPack().get_path('cleaning') + file)
    o3d.visualization.draw_geometries(resultat)

    #  Change the TCP at the end of the compute
    offline.arm.set_tcp("tcp")

    file_name = rospy.get_param("/trajectorie")
    offline.data_manager.save_var_pickle(offline.offline_trajectory, rospkg.RosPack().get_path('cleaning') + file_name)
    print("Offline strategie done")
