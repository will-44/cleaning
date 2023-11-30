#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import rospy
import rospkg
import copy
from data_manager import DataManager
from robot import Robot
from open3d_tools import Open3dTool

'''
This class generate all position available for the robot mobile base around the machine
'''


class ReachabilityRelation:
    def __init__(self, machine_path, reachability_path, poses_gap=0.20, debug=False):
        self.robot = Robot()
        self.o3d_tool = Open3dTool()
        self.point_cloud = o3d.io.read_point_cloud(machine_path)
        self.poses_gap = poses_gap
        self.poses_availables = []
        self.poses_availables_pcd = o3d.geometry.PointCloud()
        self.poses_av_tup = []

        self.robot_length = 0.3  # TODO doing test to verify this value
        self.arm_base_hight = 0.7
        self.debug = debug

        if self.debug:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
        self.base_poses_pcd = o3d.geometry.PointCloud()
        self.visible_points = o3d.geometry.PointCloud()
        self.base_poses = self.get_available_poses()
        # Machine mesh
        self.mesh = o3d.io.read_point_cloud(machine_path)
        self.reachability_map = o3d.io.read_point_cloud(reachability_path)
        self.reachability_map_vox = o3d.geometry.VoxelGrid.create_from_point_cloud(self.reachability_map,
                                                                                   voxel_size=0.05)

    def generate_poses_plan(self):
        '''
        This method generate the plan of all position possible around the machine
        :return:
        '''
        center = self.point_cloud.get_center()
        # print(center)
        # get dimension of point cloud in meter
        dim_max = self.point_cloud.get_max_bound()
        dim_min = self.point_cloud.get_min_bound()
        nb_pt_x = int((dim_max[0] + 1) / self.poses_gap)
        nb_pt_y = int((dim_max[1] + 6) / self.poses_gap)
        pos_pts_x = []
        pos_pts_y = []

        nb_pts = nb_pt_x * nb_pt_y

        for i in range(nb_pt_x):
            for k in range(nb_pt_y):
                pos_pts_y.append(k * self.poses_gap)
                pos_pts_x.append(i * self.poses_gap)

        poses_plan = np.zeros((nb_pts, 3))

        poses_plan[:, 0] = pos_pts_x
        poses_plan[:, 1] = pos_pts_y
        poses_plan[:, 2] = 0
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(poses_plan)
        trans = center - pcd.get_center()
        # trans = [0, 0, self.arm_base_hight]
        trans[2] = self.arm_base_hight
        pcd = pcd.translate(trans)
        trans = [-1.5, 0, 0]
        pcd = pcd.translate(trans, relative=True)
        return pcd

    def get_available_poses(self):
        '''
        This methode generate all the position valide around the machine
        :return:
        '''
        max_bound = self.point_cloud.get_max_bound()
        min_bound = self.point_cloud.get_min_bound()
        poses = self.generate_poses_plan()
        poses_np = np.asarray(poses.points)
        # Définissez les limites du cube
        xmin, ymin = min_bound[0] - self.robot_length, min_bound[1] - self.robot_length
        xmax, ymax = max_bound[0] + self.robot_length, max_bound[1] + self.robot_length

        # Supprimez tous les points situés à l'intérieur du cube
        mask = np.logical_and(np.logical_and(poses_np[:, 0] < xmax, poses_np[:, 0] > xmin),
                              np.logical_and(poses_np[:, 1] < ymax, poses_np[:, 1] > ymin))

        poses_ = o3d.t.geometry.PointCloud.from_legacy(poses)
        self.base_poses_pcd = poses_.select_by_mask(mask, invert=True)
        self.base_poses_pcd = o3d.t.geometry.PointCloud.to_legacy(self.base_poses_pcd)

        poses_availables = np.asarray(self.base_poses_pcd.points)
        poses_av_tup = []
        for pt in poses_availables:
            pose = (pt[0], pt[1])
            poses_av_tup.append(pose)
        return poses_av_tup

    def get_reachability_relation(self):
        '''
        Generate the reachability realation
        :return: The relation store as a dictionnary
        '''
        reachability_relation = {}
        reach_map_trans = copy.deepcopy(self.reachability_map)
        reach_map_trans.paint_uniform_color([0, 1, 0])
        self.base_poses_pcd.paint_uniform_color([1, 0, 0])
        self.mesh.paint_uniform_color([0, 0, 1])

        reach_map_trans_r = o3d.geometry.PointCloud()
        if self.debug:
            self.vis.add_geometry(reach_map_trans_r)
            self.vis.add_geometry(self.base_poses_pcd)
            # self.vis.add_geometry(self.mesh)
            self.visible_points = copy.deepcopy(self.mesh)
            self.vis.add_geometry(self.visible_points)

        for pt in self.base_poses:
            reachability_relation.update({pt: []})
        # we check all possible positions

        for pose in reachability_relation.keys():

            # self.visible_points = self.mesh
            # transform RM to actual base pose, we suppose the position as x m up to the floor and turn to orients it to the machine center
            reach_map_trans = reach_map_trans.translate([pose[0], pose[1], self.arm_base_hight], relative=False)
            angle = self.robot.mir_pose_angle(pose, self.mesh)
            # print(angle)
            reach_map_trans_r.points = reach_map_trans.points
            rot = reach_map_trans_r.get_rotation_matrix_from_xyz((0, 0, angle))
            reach_map_trans_r.rotate(rot)

            # reach_map_vox_trans = o3d.geometry.VoxelGrid.create_from_point_cloud(reach_map_trans_r,
            #                                                                      voxel_size=0.16)

            # check how much the new RM include mesh points
            # Here we consider all point in the workspace to be with quality at 1
            # TODO get the real quality and compute from that
            # OLD ONE
            # queries = np.asarray(self.mesh.points)

            # Test Voxelisation
            # downpcd = self.mesh.voxel_down_sample(voxel_size=0.05)
            # queries = np.asarray(downpcd.points)
            # Test visible points
            visible_points_new = self.get_visble_points(copy.copy(self.mesh), [pose[0], pose[1], self.arm_base_hight])
            self.visible_points.points = visible_points_new.points
            # print("update mesh")
            # queries = np.asarray(self.visible_points.points)

            # Voxel solution
            # bool_points_reachable = reach_map_vox_trans.check_if_included(o3d.utility.Vector3dVector(queries))
            #
            # # We get all point that are reachable and add it in the dict
            # bool_points_reachable = np.asarray(bool_points_reachable)
            # index_points_reachable = np.where(bool_points_reachable == True)
            # print(type(index_points_reachable))

            # Distance solution
            pcd, index_points_reachable = self.o3d_tool.compare_pcd(self.visible_points, reach_map_trans_r, precision=0.05, diff="inf")
            index_points_reachable = (np.asarray(index_points_reachable),)
            # print(index_points_reachable)
            reachability_relation[pose] = list(
                map(tuple, np.take(np.around(np.asarray(self.visible_points.points), decimals=5),
                                   index_points_reachable, axis=0)[0]))

            if self.debug:
                self.vis.update_geometry(reach_map_trans_r)
                self.vis.update_geometry(self.visible_points)
                self.vis.poll_events()
                self.vis.update_renderer()
            reach_map_trans_r.paint_uniform_color([1, 0, 0])
            # self.visible_points.paint_uniform_color([0, 1, 0])
            # hull, _ = self.mesh.compute_convex_hull()
            # hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
            # hull_ls.paint_uniform_color((1, 0, 0))
            # o3d.visualization.draw_geometries([self.base_poses_pcd, hull_ls, self.visible_points])
        return reachability_relation

    def get_visble_points(self, pcd_inital, base_pose, reach=2, interval=0.5):
        all_index = set()
        pcd_initial = copy.deepcopy(self.mesh)
        for offset in range(round(reach / interval)):
            camera = [base_pose[0], base_pose[1], base_pose[2] + offset*interval]
            radius = 7000*offset*interval+1
            _, pt_map = pcd_initial.hidden_point_removal(camera, radius)
            all_index.update(pt_map)
        result = pcd_initial.select_by_index(list(all_index))
        return result


if __name__ == '__main__':
    rospy.init_node('Reachability_relation', anonymous=True)
    debug = rospy.get_param("debug", default=True)
    print(debug)
    path_machine = rospkg.RosPack().get_path('cleaning') + rospy.get_param("machine_pcd")
    reachability_map_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("reachability_map")
    relation = ReachabilityRelation(path_machine, reachability_map_path, debug=debug)
    reachability_relation = relation.get_reachability_relation()
    print("Computing finish")
    print("Start save")
    # Save as pickle
    relation_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("relation_spot")
    data = DataManager()
    data.save_var_pickle(reachability_relation, relation_path)
    if debug:
        o3d.visualization.draw_geometries([relation.point_cloud, relation.base_poses_pcd])
    rospy.loginfo("Reachability relation finish !")
