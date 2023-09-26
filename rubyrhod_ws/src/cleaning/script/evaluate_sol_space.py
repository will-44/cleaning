#!/usr/bin/env python3
import glob
import random

import numpy as np
import open3d as o3d
import rospkg
import rospy

from data_manager import DataManager
from open3d_tools import Open3dTool
from utility.doosan import Doosan
import csv


if __name__ == '__main__':
    rospy.init_node('sol_space', anonymous=True)

    o3d_tool = Open3dTool()
    data_manager = DataManager()
    arm = Doosan()


