#!/usr/bin/env python3

import numpy as np
import open3d as o3d


class Open3dTool:

    def __init__(self, debug=False):
        self.debug = debug

    def np2pcd(self, pcd_np):
        """
        Transform a np array to a pcd
        :param pcd_np: the array
        :return: pcd
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_np)
        return pcd

    def get_pcd_density_center(self, pcd):
        # get 2d pose of all pcd points
        pcd_np = np.asarray(pcd.points)
        # mean thoses poses x, y,z
        x_mean = np.mean(pcd_np[:, 0])
        y_mean = np.mean(pcd_np[:, 1])
        z_mean = np.mean(pcd_np[:, 2])

        return [x_mean, y_mean, z_mean]
    def generate_pcd_guard(self, pcd_init, dist=0.4):
        """
        get pcd and generate guards with the same normal
        :param pcd_init: the initial point cloud
        :param dist: the distance up to the normal
        :return: the pcd for the guards points
        """
        # Set the distance of 10 cm
        distance = dist

        # Initialize arrays to store the generated points and corresponding normals
        points_sampled = []
        normals_sampled = []

        # Generate a number of random points equal to the number of points in the initial point cloud
        n_points = int(len(pcd_init.points) / 10)
        if n_points == 0:
            n_points += 1
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

        # Visualize the new point cloud
        if self.debug:
            o3d.visualization.draw_geometries([pcd_sampled])

        return pcd_sampled

    def check_obs_btw_points(self, dist_mat, guard_pts, global_mesh):
        """
        If two pts can't see each others, the distance between will be inf (1000000000)
        :param dist_mat: the distance between all points
        :param guard_pts: The points related to the distance matrix
        :param global_mesh: the collision mesh
        :return: the new distance matrix
        """

        scene = o3d.t.geometry.RaycastingScene()
        machine_mesh = o3d.t.geometry.TriangleMesh.from_legacy(global_mesh)
        cube_id = scene.add_triangles(machine_mesh)

        for idx, x in np.ndenumerate(dist_mat):
            if idx[0] != idx[1]:
                v = [guard_pts[idx[1]][0] - guard_pts[idx[0]][0],
                     guard_pts[idx[1]][1] - guard_pts[idx[0]][1],
                     guard_pts[idx[1]][2] - guard_pts[idx[0]][2]]
                v_unit = v / np.linalg.norm(v)
                rays = o3d.core.Tensor([[guard_pts[idx[0]][0], guard_pts[idx[0]][1], guard_pts[idx[0]][2],
                                         v_unit[0], v_unit[1], v_unit[2]]],
                                       dtype=o3d.core.Dtype.Float32)

                ans = scene.cast_rays(rays)

                if ans['t_hit'] != float('inf'):
                    dist_mat[idx[0], idx[1]] = 1000000000

        return dist_mat

    def remove_pt_in_collision(self, pt_cloud_main, pt_cloud_to_check, size=0.05):
        """
        Remove points from pcd near the main pcd. The default distance is 0.05
        :param pt_cloud_main: the pcd that is the base (machine...)
        :param pt_cloud_to_check: The modified pcd
        :param size: the distance
        :return: the pcd checked without points include in the main
        """
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pt_cloud_main, voxel_size=size)
        queries = np.asarray(pt_cloud_to_check.points)
        output = voxel_grid.check_if_included(o3d.utility.Vector3dVector(queries))

        xyz = np.zeros((int(len(pt_cloud_to_check.points)), 3))
        xyz = [y for x, y in zip(output, queries) if not x]
        xyz = np.array(xyz)
        pcd = self.np2pcd(xyz)

        return pcd
