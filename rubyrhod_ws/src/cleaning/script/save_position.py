#!/usr/bin/env python3
"""
This file save the position of the robot in a json file.
This is trigger from a key press in the main loop.
"""

import json
import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

from data_manager import DataManager

# Import doosan class
from utility.doosan import Doosan

# Start ROS node
if __name__ == '__main__':
    rospy.init_node('save_position', anonymous=True)
    # Global var
    data_manager = DataManager()
    robot_pose = None
    arm = Doosan()
    dic = {}

    configs = []

    # start main loop waiting a key press
    while not rospy.is_shutdown():
        input("Press Enter to save the position...")
        # get the robot configuration
        config = arm.moveit_get_configuration()

        configs.append(config)
        dic[(0, 0, 0)] = configs


    # on the close of the node hock , save the list configs in a dict with key (0, 0, 0) and save the dict in a json file
    rospy.on_shutdown(lambda: data_manager.save_dict_to_json(dic, "./test.json"))






