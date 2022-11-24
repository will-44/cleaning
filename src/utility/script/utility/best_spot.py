#!/usr/bin/env python3

import pickle
import open3d as o3d
import copy
import numpy as np
import random as rnd
import math
import matplotlib.pyplot as plt
import rospkg
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.distances import great_circle_distance_matrix, euclidean_distance_matrix


def np2pcd(xyz):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd


def remove_pt_in_collision(pt_cloud_main, pt_cloud_to_check, size=0.05):
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pt_cloud_main, voxel_size=size)
    queries = np.asarray(pt_cloud_to_check.points)
    output = voxel_grid.check_if_included(o3d.utility.Vector3dVector(queries))

    xyz = np.zeros((int(len(pt_cloud_to_check.points)), 3))
    xyz = [y for x, y in zip(output, queries) if not x]
    xyz = np.array(xyz)
    pcd = np2pcd(xyz)

    return pcd


def get_guard(pcd, pt, dist=10):
    guard = pcd.points[pt] + pcd.normals[pt] / dist
    return guard


def select_rnd_poses_norm(pcd, proba=0.01):
    # Create the pts list choose
    pcd_rnd = o3d.geometry.PointCloud()
    pts_rnd = []
    normals = np.array([[]])
    normals = np.resize(normals, (1, 3))
    points = np.array([[]])
    points = np.resize(points, (1, 3))
    # fill the list with 1/10 pts
    nb_pt = (len(pcd.points))

    # we select a point
    for pt in range(int(nb_pt * proba)):
        pt_rnd = rnd.randint(0, int(len(pcd.points)) - 1)

        # If pt already take
        if pt_rnd not in pts_rnd:
            pt_add = pcd.points[pt_rnd]
            pt_add = np.resize(pt_add, (1, 3))
            points = np.append(points, pt_add, axis=0)
            nor_add = pcd.normals[pt_rnd]
            nor_add = np.resize(nor_add, (1, 3))
            normals = np.append(normals, nor_add, axis=0)
            pts_rnd.append(pt_rnd)
    points = np.delete(points, 0, axis=0)
    normals = np.delete(normals, 0, axis=0)
    pcd_rnd.points = o3d.utility.Vector3dVector(points)
    pcd_rnd.normals = o3d.utility.Vector3dVector(normals)
    return pcd_rnd


def generate_pcd_guard(pcd):
    pcd_guards = o3d.geometry.PointCloud()
    points = np.array([[]])
    points = np.resize(points, (1, 3))
    normals = np.array([[]])
    normals = np.resize(points, (1, 3))

    for pt in range(int(len(pcd.points))):
        point = np.resize(get_guard(pcd, pt, dist=5), (1, 3))
        normal = np.resize(pcd.normals[pt], (1, 3))
        points = np.append(points, point, axis=0)
        normals = np.append(normals, normal, axis=0)

    points = np.delete(points, 0, axis=0)
    normals = np.delete(normals, 0, axis=0)
    pcd_guards.points = o3d.utility.Vector3dVector(points)
    pcd_guards.normals = o3d.utility.Vector3dVector(normals)
    return pcd_guards


def select_best_spot(relation, pcd_inital):
    flag_first = True
    list_spot = {}
    relation_sort = sorted(relation, key=lambda a: len(relation[a]), reverse=True)
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
            relation.pop(relation_sort[0])
            flag_first = False
            relation_sort = sorted(relation, key=lambda a: len(relation[a]), reverse=True)
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
        plt.pause(0.05)

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
    pcd_spot = np2pcd(best_spots)
    pcd_spot.paint_uniform_color([1, 0, 1])

    for i in range(len(pcd_spot.colors)):
        color = [rnd.uniform(0.0, 1.0), rnd.uniform(0.0, 1.0), rnd.uniform(0.0, 1.0)]
        pcd_spot.colors[i] = color
        spot_points.append(np2pcd(list(list_spot.values())[i]))

        dists = pcd_inital.compute_point_cloud_distance(spot_points[i])
        dists = np.asarray(dists)
        ind = np.where(dists < 0.1)[0]
        spot_points[i] = pcd_inital.select_by_index(ind)

        spot_points[i].paint_uniform_color(color)
    return pcd_spot, spot_points


def get_normal_vector(pcd_intial, pcds_part):
    pcd_load.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))


if __name__ == "__main__":
    # Open point cloud machine
    pcd_path = rospkg.RosPack().get_path('utility') + "/mesh/scie1.ply"
    pcd_load = o3d.io.read_point_cloud(pcd_path)
    # Generate normal vectors
    pcd_load.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

    #     Select best spot
    relation_path = rospkg.RosPack().get_path('utility') + "/data/relation.pkl"
    with open(relation_path, 'rb') as f:
        relation_reel = pickle.load(f)
    best_spots, points_spots = select_best_spot(relation_reel, pcd_load)
    print(best_spots)

    best_spots.translate((pcd_load.get_center()[0], pcd_load.get_center()[1], pcd_load.get_center()[2]))


    with open(rospkg.RosPack().get_path('utility') + "/data/best_spot.pkl", 'wb') as f:
        pickle.dump(np.asarray(best_spots.points), f)
