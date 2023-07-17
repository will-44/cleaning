#!/usr/bin/env python3

import math
import random
import sys
from turtle import Turtle

import rospkg
import tf
import rospy
from std_msgs.msg import Int64
import roslaunch
import os



if __name__ == '__main__':
    rospy.init_node('emulate_camera', anonymous=True)

    spot_limits = [20, 10]
    guard_potentials = [1]
    # guard_limits = [80, 50, 20, 10]
    guard_limits = [5, 2]
    maneuvrability_coeffs = [1]
    for spot_limit in spot_limits:
        # Change spots limit
        # create the folder
        folder_path = "/data/evaluation/" + f"{spot_limit}"
        if not os.path.exists(rospkg.RosPack().get_path('cleaning') + folder_path):
            os.mkdir(rospkg.RosPack().get_path('cleaning') + folder_path)
            # Set the compute of the best spots
            rospy.set_param('/compute_best_spots', True)
        else:
            rospy.set_param('/compute_best_spots', False)
        rospy.set_param('/spots', folder_path + "/spots.plk")
        rospy.set_param('/spots_pcds', folder_path + "/spots_pcds")


        for guard_potential in guard_potentials:
            # change guard potential
            # create the folder
            folder_path = "/data/evaluation/" + f"{spot_limit}/{guard_potential}"
            # If already exist get the actual file
            if not os.path.exists(rospkg.RosPack().get_path('cleaning') + folder_path):
                os.mkdir(rospkg.RosPack().get_path('cleaning') + folder_path)
                # Compute relation guards
                rospy.set_param('/compute_relation_guards', True)
            else:
                # Don't compute relation guards
                rospy.set_param('/compute_relation_guards', False)
            rospy.set_param('/relation_guards', folder_path + "/relation_guards")


            for guard_limit in guard_limits:
                # change guard limit
                # create the folder
                folder_path = "/data/evaluation/" + f"{spot_limit}/{guard_potential}/{guard_limit}"
                os.mkdir(rospkg.RosPack().get_path('cleaning') + folder_path)

                for maneuvrability_coeff in maneuvrability_coeffs:
                    # change guard limit
                    # create the folder
                    folder_path = "/data/evaluation/" + f"{spot_limit}/{guard_potential}/{guard_limit}/{maneuvrability_coeff}"
                    os.mkdir(rospkg.RosPack().get_path('cleaning') + folder_path)


                    # set config limit
                    rospy.set_param('/spot_limit', spot_limit)
                    rospy.set_param('guards_limit', guard_limit)
                    rospy.set_param('/potential_guard', guard_potential)
                    rospy.set_param('/maneuvrability_coeff', maneuvrability_coeff)

                    # set pcd name config
                    rospy.set_param('/guards_pcd', folder_path + "/guards_pcd")
                    rospy.set_param('/result_pcd', folder_path + "/result_pcd")
                    rospy.set_param('/observed_pcd', folder_path + "/observed_pcd")
                    rospy.set_param('/observable_concavity', folder_path + "/observable_concavity")

                    # set plk name config
                    rospy.set_param('/trajectorie', folder_path + "trajectorie.plk")

                    # launch node offline
                    node_offline = roslaunch.core.Node("cleaning", "offline.py", name="offline", namespace='/dsr01m1013',
                                                       machine_name=None, args='',
                                                       respawn=False, respawn_delay=0.0,
                                                       remap_args=None,
                                                       env_args=[("robot_description", "/dsr01m1013/robot_description")],
                                                       output="screen", cwd=None,
                                                       launch_prefix=None, required=False, filename='<unknown>')

                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()

                    process = launch.launch(node_offline)
                    while process.is_alive():
                        rospy.sleep(5)

                    # Launch node evaluation
                    node_evaluation = roslaunch.core.Node("cleaning", "evaluation.py", name="evaluation", output="screen")
                    process = launch.launch(node_evaluation)
                    while process.is_alive():
                        rospy.sleep(5)

                    rospy.set_param('/compute_best_spots', False)
                    rospy.set_param('/compute_relation_guards', False)
