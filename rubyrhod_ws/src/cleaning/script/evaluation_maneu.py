#!/usr/bin/env python3
import math
import rospkg
import rospy
from rospy_message_converter import json_message_converter
import json
import open3d as o3d
from data_manager import DataManager
from open3d_tools import Open3dTool
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from utility.doosan import Doosan

if __name__ == '__main__':
    rospy.init_node('evaluation_maneuvrability', anonymous=True)
    arm = Doosan()
    # get_theorical_dust()
    o3d_tool = Open3dTool()
    data_m = DataManager()
    worst_guard = []
    worst_j = 1
    file = rospkg.RosPack().get_path('cleaning') + "/data/evaluation_indice_manupilability/10/1/20/1trajectorie.plk"
    dict_traj = data_m.load_var_pickle(file)
    for spot in dict_traj.keys():
        if dict_traj[spot] is None:
            continue
        for guard in dict_traj[spot]:
            I = arm.get_manipulability(list(guard)) * arm.get_joint_limit_index(list(guard)) / arm.joint_index_max

            print(I)
            # print(guard)
    # worst_guard = np.asarray(worst_guard)
    # worst_guard = np.degrees(worst_guard)
    # print(worst_guard)
    # print(dict_traj)
