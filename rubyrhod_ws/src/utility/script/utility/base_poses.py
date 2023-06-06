#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import rospy
import rospkg

'''
This class generate all position available for the robot mobile base around the machine
'''
class PosesBase:
    def __init__(self, point_cloud_path, poses_gap = 0.05):
        self.point_cloud = o3d.io.read_point_cloud(point_cloud_path)
        self.poses_gap = poses_gap
        self.poses_availables = []
        self.poses_availables_pcd = o3d.geometry.PointCloud()
        self.poses_av_tup = []

    def generate_poses_plan(self):
        '''
        This method generate the plan of all position possible around the machine
        :return:
        '''
        center = self.point_cloud.get_center()
        # get dimension of point cloud in meter
        dim = self.point_cloud.get_max_bound()

        nb_pt_x = int((dim[0]+2) / self.poses_gap)
        nb_pt_y = int((dim[1]+2) / self.poses_gap)

        pos_pts_x = []
        pos_pts_y = []

        nb_pts = nb_pt_x * nb_pt_y

        for i in range(nb_pt_x):
            for k in range(nb_pt_y):
                pos_pts_y.append(k * self.poses_gap - center[0])
                pos_pts_x.append(i * self.poses_gap - center[1])


        poses_plan = np.zeros((nb_pts, 3))

        poses_plan[:, 0] = pos_pts_x
        poses_plan[:, 1] = pos_pts_y
        poses_plan[:, 2] = 0.5
        return poses_plan

    def get_available_poses(self):
        '''
        This methode generate all the position valide around the machine
        :return:
        '''
        poses = o3d.geometry.PointCloud()
        poses.points = o3d.utility.Vector3dVector(self.generate_poses_plan())

        hull, _ = self.point_cloud.compute_convex_hull()
        hull = hull.sample_points_poisson_disk(3000)
        dists = poses.compute_point_cloud_distance(hull)
        dists = np.asarray(dists)
        ind = np.where(dists > 0.4)[0]
        new_poses = poses.select_by_index(ind)
        self.poses_availables_pcd = new_poses
        self.poses_availables = np.asarray(new_poses.points)
        self.convert2tuple()
        return self.poses_availables

    def convert2tuple(self):
        '''
        this method convert the positions to tuple, we consider the z axe constant
        :return: a list of tuple
        '''
        for pt in self.poses_availables:
            pose = (pt[0], pt[1])
            self.poses_av_tup.append(pose)

    def visualize_poses_available(self):
        '''
        Debug method to visualize the available position
        :return: open a windows open3d
        '''
        poses = o3d.geometry.PointCloud()
        poses.points = o3d.utility.Vector3dVector(self.get_available_poses())
        o3d.visualization.draw_geometries([poses])

if __name__ == '__main__':
    rospy.init_node('reachability', anonymous=True)
    path = rospkg.RosPack().get_path('utility') + "/mesh/scie1.ply"
    poses = PosesBase(path)
    poses.visualize_poses_available()

    pcd_poses = o3d.geometry.PointCloud()
    pcd_poses.points = o3d.utility.Vector3dVector(poses.get_available_poses())
    o3d.visualization.draw_geometries([poses.point_cloud, pcd_poses])





