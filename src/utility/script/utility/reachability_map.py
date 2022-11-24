#!/usr/bin/env python3
import math
import pickle

import geometry_msgs.msg
import numpy as np
from doosan import Doosan
import rospy
import open3d as o3d
from alive_progress import alive_bar
import rospkg
from base_poses import PosesBase
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from rviz_tool import display_marker
from visualization_msgs.msg import Marker
'''
This class generate the reachability map of the doosan. 
It can save it as a point cloud (pcd) or load it from a pcd
'''


class ReachabilityMap:

    def __init__(self):

        # The arm controler
        self.arm = Doosan()

        # The reach of the doosan is 1.3m from the constructor
        self.doosan_reach = 1.3

        # We set the gap between points to reach at 0.05m. More would be longer to generate.
        self.point_gap = 0.05

        # The point cloud of the reachability map
        self.point_cloud = o3d.geometry.PointCloud()

        # We create the initial reachability map. We add 0.5m to be sure to explore the all workspace of the robot,
        # even the constructor say 1.3m it can be a bit more.
        nb_pt_x = int((self.doosan_reach + 0.5) / self.point_gap)
        nb_pt_y = int((self.doosan_reach + 0.5) / self.point_gap)
        nb_pt_z = int((self.doosan_reach + 0.5) / self.point_gap)

        pos_pts_x = []
        pos_pts_y = []
        pos_pts_z = []

        # The initial number of points
        self.nb_pt_initial = nb_pt_x * nb_pt_y * nb_pt_z

        # We generate all points around the robot
        for i in range(int(-nb_pt_x / 2), int(nb_pt_x / 2)):
            for j in range(int(-nb_pt_y / 2), int(nb_pt_y / 2)):
                for k in range(int(-nb_pt_z / 2), int(nb_pt_z / 2)):
                    pos_pts_y.append(j * self.point_gap)
                    pos_pts_x.append(i * self.point_gap)
                    pos_pts_z.append(k * self.point_gap)

        # The reachability map of the robot set to 0
        self.reachability_map = np.zeros((self.nb_pt_initial, 3))
        self.reachability_map[:, 0] = pos_pts_x
        self.reachability_map[:, 1] = pos_pts_y
        self.reachability_map[:, 2] = pos_pts_z
        self.capacity_map = np.zeros((self.nb_pt_initial, 4))
        self.capacity_map[:, 0] = pos_pts_x
        self.capacity_map[:, 1] = pos_pts_y
        self.capacity_map[:, 2] = pos_pts_z
        self.capacity_map[:, 3] = 1

    def get_reachability_map(self):
        '''
        Ths method check all point from the theorical reachability map and generate the real reachability map
        This method is long to compute and is blocking, it should be done only one time
        :return: the point cloud of the reachability map
        '''
        new_map = []
        pt_add = 0
        with alive_bar(self.nb_pt_initial) as bar:
            # parcour les pt pour chaque position
            for index in range(np.shape(self.reachability_map)[0]):
                # TODO generate a better orientation
                q = quaternion_from_euler(math.atan2(self.reachability_map[index][1], self.reachability_map[index][2]),
                                          math.atan2(self.reachability_map[index][2], self.reachability_map[index][0]),
                                          math.atan2(self.reachability_map[index][0], self.reachability_map[index][1]))
                pose2reach = Pose()
                pose2reach.position.x = self.reachability_map[index][0]
                pose2reach.position.y = self.reachability_map[index][1]
                pose2reach.position.z = self.reachability_map[index][2]
                pose2reach.orientation.x = q[0]
                pose2reach.orientation.y = q[1]
                pose2reach.orientation.z = q[2]
                pose2reach.orientation.w = q[3]
                ikin = self.arm.ikin(pose2reach)

                if ikin.error_code.val == 1:
                    # print(ikin)
                    # colision = self.arm.check_collision(ikin.solution.joint_state.position).valid
                    # print(colision)
                    # print(pose2reach)
                    # if colision:
                    #     print("accessible")
                    #     display_marker(Marker.ARROW,
                    #                    pose2reach.position.x,
                    #                    pose2reach.position.y,
                    #                    pose2reach.position.z,
                    #                    pose2reach.orientation.x,
                    #                    pose2reach.orientation.y,
                    #                    pose2reach.orientation.z,
                    #                    pose2reach.orientation.w, 'world')
                    #     input()
                    new_map.append(self.reachability_map[index])
                        # # we set the quality of the reachable point to 1
                        # # TODO calculate a more accurate quality value
                        # self.capacity_map[index][3] = 1
                        # pt_add += 1
                        # self.arm.go_to_l(pose2reach)
                        # self.arm.go_to_j(list(ikin.solution.joint_state.position))
                        # while self.arm.check_motion() != 0:
                        #     rospy.sleep(0.1)
                            # print(pt_add)
                bar()
        # print(np.shape(self.reachability_map))
        self.reachability_map = np.array(new_map)
        # print(np.shape(self.reachability_map))
        return self.reachability_map

    def save_map_as_pcd(self, path):
        '''
        This method save the reachability map to a pcd un .ply format
        :param path: string: the path to store the pcd
        :return:
        '''
        self.point_cloud.points = o3d.utility.Vector3dVector(self.reachability_map)
        o3d.io.write_point_cloud(path, self.point_cloud)

    def load_map_as_pcd(self, path):
        '''
        This method load the reachability map from a pcd
        :param path:string: the path to load the pcd
        :return:
        '''
        self.point_cloud = o3d.io.read_point_cloud(path)
        return self.point_cloud


'''
This class generate the reacahability realation
'''


class ReachabilityRelation:
    def __init__(self, base_poses, targets, reachability_map):
        '''

        :param base_poses: All poses available for the robot as a list of tuple (x, y)
        :param targets: point cloud of all points targets in the machine
        :param reachability_map: point cloud of all poses available for the robot arm
        '''
        self.base_poses = base_poses
        self.targets = targets
        self.reachability_map = reachability_map
        self.reachability_map_vox = o3d.geometry.VoxelGrid.create_from_point_cloud(self.reachability_map,
                                                                                   voxel_size=0.05)
        self.reachability_relation = {}
        self.quality_threshold = 1

    def get_reachability_relation(self):
        '''
        Generate the reachability realation
        :return: The relation store as a dictionnary
        '''

        # debug
        reach_map_trans = self.reachability_map
        vis.add_geometry(reach_map_trans)
        vis.add_geometry(glob_pose)
        vis.add_geometry(self.targets)
        # end debug

        for pt in self.base_poses:
            self.reachability_relation.update({pt: []})
        # we check all possible positions
        for pose in self.reachability_relation.keys():
            # transform RM to actual base pose, we suppose the position as 1m up to the floor
            reach_map_trans = self.reachability_map.translate([pose[0], pose[1], 1], relative=False)
            reach_map_vox_trans = o3d.geometry.VoxelGrid.create_from_point_cloud(reach_map_trans,
                                                                                 voxel_size=0.05)
            # debug
            vis.update_geometry(reach_map_trans)
            vis.poll_events()
            vis.update_renderer()
            # end debug

            # check how much the new RM include targets points
            # Here we consider all point in the workspace to be with quality at 1
            # TODO get the real quality and compute from that
            queries = np.asarray(self.targets.points)
            bool_points_reachable = reach_map_vox_trans.check_if_included(o3d.utility.Vector3dVector(queries))

            # We get all point that are reachable and add it in the dict
            bool_points_reachable = np.asarray(bool_points_reachable)
            index_point_reachable = np.where(bool_points_reachable == True)

            # print (np.take(np.asarray(self.targets.points), index_point_reachable, axis=0))
            self.reachability_relation[pose] = tuple(
                map(tuple, np.take(np.asarray(self.targets.points), index_point_reachable, axis=0)[0]))

        return self.reachability_relation


if __name__ == '__main__':
    global glob_pose, vis
    rospy.init_node('reachability', anonymous=True)

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()

    reach = ReachabilityMap()
    reachability_map = reach.get_reachability_map()


'''
    path_map = rospkg.RosPack().get_path('utility') + "/mesh/reachability_map.ply"
    reach.save_map_as_pcd(path_map)

    # o3d.visualization.draw_geometries([reach.point_cloud])
    reachability_map = reach.load_map_as_pcd(path_map)

    path_pcd = rospkg.RosPack().get_path('utility') + "/mesh/scie1.ply"
    poses_base = PosesBase(path_pcd)
    # Get all poses available around
    poses_base.get_available_poses()
    # Get poses available in pcd and tuple
    glob_pose = poses_base.poses_availables_pcd
    poses_available = poses_base.poses_av_tup

    # Get all target point on mesh
    targets = o3d.io.read_point_cloud(path_pcd)
    targets = targets.voxel_down_sample(voxel_size=0.05)

    relation = ReachabilityRelation(base_poses=poses_available, reachability_map=reachability_map, targets=targets)

    with open(rospkg.RosPack().get_path('utility') + '/script/utility/relation.pkl', 'wb') as f:
        pickle.dump(relation.get_reachability_relation(), f)
'''
# with open('saved_dictionary.pkl', 'rb') as f:
#     loaded_dict = pickle.load(f)
