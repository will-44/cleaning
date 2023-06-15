#!/usr/bin/env python3
import copy
import math
import time
from math import radians

import numpy as np
import open3d as o3d
import rospkg
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped
from open3d_ros_helper import open3d_ros_helper as o3d_ros
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray


class FovEmulator:
    def __init__(self):
        # rospy.Subscriber("/transf_mat", Float64MultiArray, self.callback)
        self.result_pub = rospy.Publisher('/pcl_result', PointCloud2, queue_size=10)
        # Load the machine in pcd
        self.machine_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_pcd")
        self.machine_pcd = o3d.io.read_point_cloud(self.machine_path)

        self.machine_pcd_initial = self.machine_pcd

        # Generate the cone parameters
        # This cone is generate along the X axis because the frame of the camera is alog this axis
        #
        #                ^ cone_alpha
        #               /│\
        #              / │ \
        #             /  │  \
        #            /   │   \
        #           /    │    \
        #          /     │     \
        #         /      │      \cone_surface
        #        /       │       \
        #       /        │        \
        #      /         │cone_x   \
        #     /          │          \
        #    /           │           \
        #   /            │            \
        #  /             │             \
        # /──────────────┴──────────────\
        #                     cone_r

        self.cone_x = 0.5
        self.cone_alpha = radians(30)  # fov min /2
        self.cone_r = self.cone_x * math.tan(self.cone_alpha)
        self.cone_surface = self.cone_r / math.sin(self.cone_alpha)

        self.cone_np = np.array([[0, 0, 0], [self.cone_x, 0, 0]])
        self.cone_pcd = o3d.geometry.PointCloud()
        self.cone_pcd.points = o3d.utility.Vector3dVector(self.cone_np)

        # Debug
        self.debug = rospy.get_param("debug", default=False)

        if self.debug:
            mesh_cone = o3d.geometry.TriangleMesh.create_cone(radius=self.cone_r, height=self.cone_x)
            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
            o3d.visualization.draw_geometries([mesh, mesh_cone])

            R = mesh_cone.get_rotation_matrix_from_xyz((np.pi, 0, 0))
            mesh_cone.rotate(R, center=(0, 0, 0))
            mesh_cone.translate((0, 0, self.cone_x))
            o3d.visualization.draw_geometries([mesh, mesh_cone])

            R = mesh_cone.get_rotation_matrix_from_xyz((0, np.pi / 2, 0))
            mesh_cone.rotate(R, center=(0, 0, 0))

            o3d.visualization.draw_geometries([mesh, mesh_cone])

    def get_visible_point(self, trans):

        machine_pcd = copy.deepcopy(self.machine_pcd_initial)
        start_time = time.time()
        # open cone pcd + machine pcd
        mat = np.asarray(trans.data)
        mat.resize(4, 4)
        # print(mat)

        # cone is a pcd with 2pts as the top of the cone and the bot
        cone_t = copy.deepcopy(self.cone_pcd).transform(mat)
        cone_np = np.asarray(self.cone_pcd.points)
        cone_t_np = np.asarray(cone_t.points)

        # Setup camera pose
        camera = cone_t_np[0]

        # Get all points visible by the camera
        radius = 100
        # diameter = np.linalg.norm(
        #     np.asarray(machine_pcd.get_max_bound()) - np.asarray(machine_pcd.get_min_bound()))
        # radius = diameter * 10
        _, pt_map = machine_pcd.hidden_point_removal(camera, radius)
        machine_pcd = machine_pcd.select_by_index(pt_map)
        machine_np = np.asarray(machine_pcd.points)

        if self.debug:
            cone_t.paint_uniform_color([0, 0, 1])
            # machine_pcd.paint_uniform_color([1, 0, 0])
            mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.06, origin=[0, 0, 0])

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            trans_cone = tf_buffer.lookup_transform('cone', 'machine', rospy.Time(),
                                                    rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo("FOV_EMULATOR: pb dans la transformation")
            pcd = o3d.geometry.PointCloud()
            self.result_pub.publish(pcd)
            return

        try:
            trans_machine = tf_buffer.lookup_transform('machine', 'cone', rospy.Time(),
                                                       rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo("FOV_EMULATOR: pb dans la transformation")
            pcd = o3d.geometry.PointCloud()
            self.result_pub.publish(pcd)
            return

        pts_include = []
        # print(cone_np)
        # for each pts in the machine we create the 2 vector: cone a->b and a->pt
        # with the dot product we get the angle and then the norme of the two
        for pt in machine_np:

            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = pt[2]
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            pt = tf2_geometry_msgs.do_transform_pose(pose, trans_cone)

            norm_origine = math.sqrt(pow(cone_np[0][0] - pt.pose.position.x, 2) +
                                     pow(cone_np[0][1] - pt.pose.position.y, 2) +
                                     pow(cone_np[0][2] - pt.pose.position.z, 2))

            norm_base = math.sqrt(pow(cone_np[1][0] - pt.pose.position.x, 2) +
                                  pow(cone_np[1][1] - pt.pose.position.y, 2) +
                                  pow(cone_np[1][2] - pt.pose.position.z, 2))
            # If the point is in front of the cone to avoid the double cone
            # Ca ne resoud rien, ou pas ?
            if norm_base <= self.cone_x and norm_origine <= self.cone_surface:
                # For this equation we swap the X and Z axis, no clue why
                eq_cone = pow(pt.pose.position.z - cone_np[0][2], 2) + \
                          pow(pt.pose.position.y - cone_np[0][1], 2) - \
                          (pow(pt.pose.position.x - cone_np[0][0], 2) * pow((math.tan(self.cone_alpha)), 2))

                if eq_cone <= 0:
                    pt = tf2_geometry_msgs.do_transform_pose(pt, trans_machine)
                    pts_include.append([pt.pose.position.x, pt.pose.position.y, pt.pose.position.z])

        pts_include_np = np.asarray(pts_include)

        pts_include_pcd = o3d.geometry.PointCloud()
        if pts_include:
            pts_include_pcd.points = o3d.utility.Vector3dVector(pts_include_np)
            if self.debug:
                print("--- %s seconds ---" % (time.time() - start_time))
                pts_include_pcd.paint_uniform_color([1, 0, 0])

                cone_t = copy.deepcopy(self.cone_pcd).transform(mat)
                mesh = o3d.geometry.TriangleMesh.create_coordinate_frame().transform(mat)
                mesh_cone = o3d.geometry.TriangleMesh.create_cone(radius=self.cone_r, height=self.cone_x)
                mesh_cone_t = copy.deepcopy(mesh_cone).transform(mat)

                cone_t.paint_uniform_color([0, 0, 1])
                o3d.visualization.draw_geometries([machine_pcd, pts_include_pcd, cone_t, mesh_cone_t])

        # print(pts_include_np)
        pcd = o3d_ros.o3dpc_to_rospc(pts_include_pcd)
        pcd.header.frame_id = "machine"
        self.result_pub.publish(pcd)
        return pcd


if __name__ == '__main__':
    rospy.init_node('emulate_camera', anonymous=True)

    rospy.spin()
