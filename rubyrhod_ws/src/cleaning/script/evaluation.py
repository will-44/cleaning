#!/usr/bin/env python3
import glob
import random

import numpy as np
import open3d as o3d
import rospkg
import rospy

from data_manager import DataManager
from open3d_tools import Open3dTool

import csv


def save_csv(coverage_initial, overflap_initial, coverage_concavity, overlap_concavity, nb_guard, nb_guard_concavity,
             nb_spots):
    csv_path = rospkg.RosPack().get_path('cleaning') + "/data/evaluation/evaluation.csv"
    with open(csv_path, 'a') as f:
        # create the csv writer
        writer = csv.writer(f)

        row = [rospy.get_param('/spot_limit'), rospy.get_param('guards_limit'), rospy.get_param('/potential_guard'),
               nb_spots, nb_guard, coverage_initial, overflap_initial,
               nb_guard_concavity, coverage_concavity, overlap_concavity]
        # write a row to the csv file
        writer.writerow(row)
    return


def evaluation(initial_pcd, result_pcd):
    # Calcul porcentage correspondance
    dists = initial_pcd.compute_point_cloud_distance(result_pcd)
    dists = np.asarray(dists)
    ind = np.where(dists < 0.0001)[0]
    cummun_points = initial_pcd.select_by_index(ind)
    cummun_points.paint_uniform_color((0, 1, 0))
    obs = len(np.asarray(result_pcd.points))
    pts_mesh = len(np.asarray(initial_pcd.points))
    commun = len(ind)
    # ind = np.where(dists > 0.0001)[0]
    # pcd = initial_pcd.select_by_index(ind)
    covering_conv = (commun / pts_mesh) * 100
    overlap_conv = ((obs - commun) / pts_mesh) * 100
    return covering_conv, overlap_conv


if __name__ == '__main__':
    rospy.init_node('evaluation', anonymous=True)

    o3d_tool = Open3dTool()
    data_manager = DataManager()
    # Load machine pcd
    machine_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param(
        "/machine_pcd")  # "mesh/scie_3.ply"  #machine_pcd
    machine_pcd = o3d.io.read_point_cloud(machine_path)

    file_spot = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/spots_pcds")
    # nomber of spots
    file_names = glob.glob(f"{file_spot}_*.pcd")
    nb_spots = len(file_names)
    file = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/observed_pcd")
    pcds = data_manager.load_pcd_list(file)
    pcds_concavity = data_manager.load_pcd_list(
        rospkg.RosPack().get_path('cleaning') + rospy.get_param("/observable_concavity"))

    # Fuse the pcd observable
    final_pcd = o3d.geometry.PointCloud()
    nb_guard = 0
    for pcd in pcds:
        pcd_np = np.asarray(pcd.points)
        nb_guard += 1  # len(pcd_np)
        final_pcd_np = np.asarray(final_pcd.points)

        # give colors
        pcd.paint_uniform_color([random.uniform(0.0, 1.0), random.uniform(0.0, 1.0), random.uniform(0.0, 1.0)])

        p3_load = np.concatenate((pcd_np, final_pcd_np), axis=0)
        final_pcd.points = o3d.utility.Vector3dVector(p3_load)

    final_pcd.paint_uniform_color([0, 0, 1])

    final_pcd_np = np.asarray(final_pcd.points)

    # Calcul porcentage correspondance
    covering, overlap = evaluation(machine_pcd, final_pcd)

    # With concavity
    concavity_pcd = o3d.geometry.PointCloud()
    nb_guard_conv = 0
    for pcd in pcds_concavity:
        pcd_np = np.asarray(pcd.points)
        nb_guard_conv += 1
        concavity_pcd_np = np.asarray(concavity_pcd.points)

        p3_load = np.concatenate((pcd_np, concavity_pcd_np), axis=0)
        concavity_pcd.points = o3d.utility.Vector3dVector(p3_load)

    all_pcd = o3d.geometry.PointCloud()
    all_pcd_np = np.concatenate((np.asarray(concavity_pcd.points), final_pcd_np), axis=0)

    all_pcd.points = o3d.utility.Vector3dVector(all_pcd_np)

    covering_conv, overlap_conv = evaluation(machine_pcd, all_pcd)

    save_csv(covering, overlap, covering_conv, overlap_conv, nb_guard, nb_guard_conv, nb_spots)
