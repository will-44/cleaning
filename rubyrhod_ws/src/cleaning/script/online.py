#!/usr/bin/env python3

import pickle
from copy import copy

import geometry_msgs.msg
import numpy as np
import rospkg
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose, Pose2D
from open3d_ros_helper import open3d_ros_helper as o3d_ros
from ros_numpy import numpify
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from utility.doosan import Doosan
from utility.rviz_tool import display_marker_array, display_text_array, reset_rviz
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
from data_manager import DataManager
from open3d_tools import Open3dTool
from robot import Robot
from dust import Dust
from tf_tools import TfTools
from utility.srv import DetectDust
from python_tsp.distances import euclidean_distance_matrix


class OnlineStrategy:
    def __init__(self, path_machine, offline_trajectory_path):
        # Callback ROS
        # self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback_color)
        # self.depth_sub = rospy.Subscriber("/depth_to_rgb/image_raw", Image, self.callback_depth)

        # Publisher
        self.pcl_pub = rospy.Publisher('/transf_mat', Float64MultiArray, queue_size=10)

        # Pub telemetrie
        self.guard_pub = rospy.Publisher('/telemetrie/guard', JointState, queue_size=10)
        self.dust_pub = rospy.Publisher('/telemetrie/dust_pose', PoseStamped, queue_size=10)
        self.finish_pub = rospy.Publisher('/telemetrie/finish', Bool, queue_size=10)
        self.octo_pub = rospy.Publisher('/toggle_octomap', Bool, queue_size=10)
        self.spot_pub = rospy.Publisher('/telemetrie/spot', Pose2D, queue_size=10)

        # Open json file
        self.data_manager = DataManager()
        print(offline_trajectory_path)
        self.offline_traj = self.data_manager.load_json_to_dict(offline_trajectory_path)

        # Dicts (old version, with diciotnary saved from pickel)
        # with open(offline_trajectory_path, 'rb') as f:
        #     self.offline_traj = pickle.load(f)

        # Global var
        self.base_trajectory = list(map(list, list(self.offline_traj.keys())))
        print(self.base_trajectory)
        # self.depth_image = None
        # self.color_image = None

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.broadcaster_robot = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.arm = Doosan()
        self.data_manager = DataManager()
        self.open3d_tool = Open3dTool()
        self.robot = Robot()
        self.dust = Dust()
        self.tf_tools = TfTools()
        print("online ready")

    # def callback_color(self, img):
    #     self.color_image = img
    #
    # def callback_depth(self, img):
    #     self.depth_image = img


if __name__ == '__main__':
    global color_image, depth_image
    rospy.init_node('online_strategie', anonymous=True)

    traj_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/trajectorie")
    machine_pcd_path = rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_pcd")

    debug = rospy.get_param("/debug", default=False)

    broadcaster_guard = tf2_ros.StaticTransformBroadcaster()
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    strat = OnlineStrategy(machine_pcd_path, traj_path)
    rospy.sleep(1)
    is_sim = True
    dont_move = False
    is_detect_dust = True

    index_mir = 0
    index = input("from witch index do you want to start ?")

    for spot in strat.base_trajectory[int(index):]:
        print(spot[0], spot[1], spot[2])
        strat.arm.go_home()
        # while strat.arm.check_motion() != 0:
        #     rospy.sleep(0.8)

        answ = input("Are you ready to go to the next spot ?")
        if answ == "no":
            pass
        if dont_move:
            result_mir = True
            spot_pose = Pose2D()
            spot_pose.x = spot[0]
            spot_pose.y = spot[1]
            spot_pose.theta = spot[2]
            strat.spot_pub.publish(spot_pose)
        else:
            if is_sim:
                result_mir, trans_machine = strat.robot.move_mobile_base(spot[0], spot[1], spot[2])
                result_mir = True
                # rospy.sleep(10)
                collision_stamp = PoseStamped()
                collision_stamp.header.frame_id = "machine"
                collision_stamp.pose.orientation.w = 1
                try:
                    trans_machine = tf_buffer.lookup_transform('world', "machine", rospy.Time(),
                                                               rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    rospy.loginfo("pb dans la transformation")

                collision_stamp = tf2_geometry_msgs.do_transform_pose(collision_stamp, trans_machine)
                print("continue")
            else:
                result_mir = strat.robot.move_base(spot[0], spot[1], spot[2])
        # result_mir = False
        if result_mir:
            input("The mir arrived, does the vacuum ready ?")
            # # clear la octomap

            # activate octomap
            strat.octo_pub.publish(True)

            strat.arm.set_tcp("vacuum_tcp")

            arm_poses = strat.offline_traj[tuple(spot)]
            print(type(arm_poses))
            print(strat.offline_traj)

            strat.arm.set_tcp("camera_base")
            # Go to guards
            if arm_poses is None:
                continue
            answer = input("select the guards ?")
            for index, pose in enumerate(arm_poses[int(answer):]):
                index = int(answer) + index
                # clear la octomap
                # strat.arm.clear_collisions()


                rospy.loginfo('{nodeName} : Position pour le GUARD'.format(nodeName=rospy.get_name()))
                print(pose)
                rospy.loginfo('{nodeName} : GUARD numero: %d'.format(nodeName=rospy.get_name()), index)
                res = strat.arm.go_to_j(pose)

                config = JointState()
                config.position = pose
                if res:
                    config.velocity = [10, 10, 10, 10, 10, 10]
                else:
                    config.velocity = [0, 0, 0, 0, 0, 0]
                config.header.stamp = rospy.get_rostime()

                strat.guard_pub.publish(config)

                # while strat.arm.check_motion() != 0:
                #     rospy.sleep(0.1)
                rospy.sleep(2)
                # GO TO DUST
                if is_detect_dust: # and res: #dire que res est toujours vrai
                    print("mouvement reach !")
                    # Set the TCP at the end of vacuum, and we add 5cm to set up an approche point.
                    # the rest will be executed by compliance
                    strat.arm.set_tcp("vacuum_safe_tcp")
                    strat.arm.dsr_set_tcp("vacuum")

                    # Detect Dust in vacuum_tcp frame
                    dust_poses = strat.dust.ask_dust_poses()

                    # Filter points
                    print(dust_poses)
                    dust_poses = strat.dust.filter_dust(dust_poses, dist=0.5)
                    print("after filter")
                    print(dust_poses)
                    # Compute the path thought dust clusters in vacuum_tcp frame
                    entry_points, dust_path = strat.dust.dust_path(dust_poses)
                    # input("Press enter to continue:")
                    # print("entry_points:")
                    # print(entry_points)
                    # print("dust_path:")
                    # print(dust_path)
                    # On a les entry point de chaque cluster et le path de chaque cluster
                    # Donc on peut se deplacer a l'entry_point avec moveit et utiliser le reste avec dsr
                    # en recuperant les orientations du entry point final,
                    # cad celui qui a ete realisé en fct des obstacles.

                    #  Define the oriantation and position for the entry pose to vacuum the dust

                    tcp_pose = strat.arm.get_pose()
                    trans_tcp = strat.tf_tools.get_transform("world", "vacuum_tcp")

                    tcp_pose_world = tf2_geometry_msgs.do_transform_pose(tcp_pose, trans_tcp)

                    trans_world = strat.tf_tools.get_transform("vacuum_tcp", "world")

                    entry_point_goals = []

                    # Def orien et pos pour chaque entry_points in the world
                    # (because pathplanning is always for world frame)
                    for dust_cluster in entry_points:
                        pose = tcp_pose_world
                        pose.pose.position.x = dust_cluster[0]
                        pose.pose.position.y = dust_cluster[1]
                        pose.pose.position.z = dust_cluster[2]
                        goal_pose = tf2_geometry_msgs.do_transform_pose(pose, trans_world)
                        entry_point_goals.append(goal_pose)

                    dust_path_world = []
                    for dust_cluster in dust_path:
                        # Transform the dust path in world frame
                        dust_path_world.append(strat.tf_tools.transform_pose_array(
                            strat.tf_tools.array_2_posestamped(dust_cluster), trans_world))

                    reset_rviz()
                    display_marker_array(Marker.SPHERE, dust_poses, "vacuum_tcp")

                    # input("look for sphere on rviz:")
                    # On deplace le robot a chaque entry_point
                    for i, entry in enumerate(entry_point_goals):

                        display_text_array(dust_path_world[i], "world")
                        print("Go to next entry point:")
                        print(entry.pose)
                        res = strat.arm.go_to_l(entry.pose)

                        # If res is false, try an other point from the entry_points's dust_path_world
                        if not res:
                            for new_point in dust_path_world[i]:
                                print("Go to next NEW entry point:")
                                res = strat.arm.go_to_l(new_point.pose)
                                if res:
                                    break

                        if res:
                            # Get orientation of the current tcp pose in
                            tcp_pose = strat.arm.dsr_get_pose()
                            print("Set compliance")
                            strat.arm.set_compliance([100, 100, 100, 20, 20, 20])
                            # strat.arm.set_force([0, 0, -1, 0, 0, 0], [0, 0, 1, 0, 0, 0])
                            reverse_path = []
                            first_dust_pose = []
                            for index, dust in enumerate(dust_path_world[i]):
                                # tcp_pose orientation is in rad euler zyz
                                dust_pose = [dust.pose.position.x, dust.pose.position.y, dust.pose.position.z,
                                             tcp_pose[3], tcp_pose[4], tcp_pose[5]]
                                reverse_path.append(copy(dust_pose))
                                print(reverse_path)
                                if index == 0:
                                    print("save first point")
                                    first_dust_pose = copy(dust_pose)
                                print("Go to next dust:")
                                print(dust_pose)
                                res = strat.arm.dsr_go_to_l(dust_pose)
                                # if the dust path got only one element or if it`s the last of dust_path_world, do a spiral
                                if index == len(dust_path_world[i]) - 1:
                                    strat.arm.dsr_spiral(revolution=2, max_radius=10, time=5)
                                # send dust goal to telemetrie
                                pose = PoseStamped()
                                pose.pose = dust.pose
                                pose.header.stamp = rospy.get_rostime()
                                # Real frame is "world"
                                if res:
                                    pose.header.frame_id = "success"
                                else:
                                    pose.header.frame_id = "fail"
                                strat.dust_pub.publish(pose)
                            # Go to the first point to avoid collision and exit from the same entry path
                            print("reverse path:")
                            print(reverse_path)
                            reverse_path.reverse()
                            print("go to first pose to extract")
                            res = strat.arm.dsr_go_to_l(first_dust_pose)
                            # for pose_exit in reverse_path:
                            #     res = strat.arm.dsr_go_to_l(pose_exit)
                            #     print("reverse path")
                            #     rospy.sleep(2)
                            print("exit from cluster")
                            # we go out
                            if res:
                                res = strat.arm.dsr_go_relatif([0, 0, -75, 0, 0, 0])

                            #print("exit from cluster")
                            # Stop compliance
                            print("Release compliance")
                            # strat.arm.release_force()
                            strat.arm.release_compliance()

                        # STOP GO TO DUST
                        reset_rviz()

            # deactivate octomap
            strat.octo_pub.publish(False)
            # clear la octomap
            # strat.arm.clear_collisions()
            # strat.arm.clear_collisions()

    # input("Cleanning finish, press enter to continue:")
    strat.finish_pub.publish(True)
    strat.arm.go_home()
