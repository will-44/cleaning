#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import rospy
import rospkg
import copy
from data_manager import DataManager

'''
This class generate all position available for the robot mobile base around the machine
'''


class ReachabilityRelation:
    def __init__(self, machine_path, reachability_path, poses_gap=0.05, debug=False):
        self.point_cloud = o3d.io.read_point_cloud(machine_path)
        self.poses_gap = poses_gap
        self.poses_availables = []
        self.poses_availables_pcd = o3d.geometry.PointCloud()
        self.poses_av_tup = []

        self.robot_length = 0.7  # TODO doing test to verify this value
        self.arm_base_hight = 0.7
        self.debug = debug

        if self.debug:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
        self.base_poses_pcd = o3d.geometry.PointCloud()
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
        nb_pt_x = int((dim_max[0] + 4) / self.poses_gap)
        nb_pt_y = int((dim_max[1] + 4) / self.poses_gap)

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
        trans[2] = self.arm_base_hight
        pcd = pcd.translate(trans)

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
        reach_map_trans.paint_uniform_color([0,1,0])
        self.base_poses_pcd.paint_uniform_color([1,0,0])
        self.mesh.paint_uniform_color([0,0,1])
        if self.debug:
            self.vis.add_geometry(reach_map_trans)
            self.vis.add_geometry(self.base_poses_pcd)
            self.vis.add_geometry(self.mesh)

        for pt in self.base_poses:
            reachability_relation.update({pt: []})
        # we check all possible positions
        for pose in reachability_relation.keys():
            # transform RM to actual base pose, we suppose the position as 1m up to the floor
            reach_map_trans = reach_map_trans.translate([pose[0], pose[1], self.arm_base_hight], relative=False)
            reach_map_vox_trans = o3d.geometry.VoxelGrid.create_from_point_cloud(reach_map_trans,
                                                                                 voxel_size=0.08)

            if self.debug:
                self.vis.update_geometry(reach_map_trans)
                self.vis.poll_events()
                self.vis.update_renderer()

            # check how much the new RM include mesh points
            # Here we consider all point in the workspace to be with quality at 1
            # TODO get the real quality and compute from that
            queries = np.asarray(self.mesh.points)
            bool_points_reachable = reach_map_vox_trans.check_if_included(o3d.utility.Vector3dVector(queries))

            # We get all point that are reachable and add it in the dict
            bool_points_reachable = np.asarray(bool_points_reachable)
            index_point_reachable = np.where(bool_points_reachable == True)

            reachability_relation[pose] = list(
                map(tuple, np.take(np.around(np.asarray(self.mesh.points), decimals=5),
                                   index_point_reachable, axis=0)[0]))

        return reachability_relation


if __name__ == '__main__':
    rospy.init_node('Reachability_relation', anonymous=True)
    debug = rospy.get_param("debug", default=False)
    path_machine = rospkg.RosPack().get_path('cleaning') + rospy.get_param("machine_pcd")
    reachability_map_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("reachability_map")
    relation = ReachabilityRelation(path_machine, reachability_map_path, debug=debug)
    reachability_relation = relation.get_reachability_relation()

    # Save as pickle
    relation_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("relation_spot")
    data = DataManager()
    data.save_var_pickle(reachability_relation, relation_path)
    if debug:
        o3d.visualization.draw_geometries([relation.point_cloud, relation.base_poses_pcd])
    rospy.loginfo("Reachability relation finish !")