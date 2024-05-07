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

import rospkg
from rospy_message_converter import json_message_converter
import json
import open3d as o3d
from data_manager import DataManager
from open3d_tools import Open3dTool
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# Fonction pour convertir un JSON en message ROS
def convert_to_ros_message(json_data, message_type):
    return json_message_converter.convert_json_to_ros_message(message_type, json_data)


def get_trans_mat(poses):
    rot = R.from_euler('xyz', [0, 0, -poses[2]]) #pourquoi un moins !!!!!
    rot = rot.as_matrix()
    mat = np.column_stack((rot[0], rot[1], rot[2], [poses[0], poses[1], 0.7]))
    mat = np.append(mat, [[0, 0, 0, 1]], axis=0)
    return mat

def get_theorical_dust():
    o3d_tool = Open3dTool()
    artag_pose = [-0.08336138881899191, 0.9152489689435612, 1.0366598910813083]
    artag = o3d_tool.np2pcd(np.asarray([artag_pose]))
    mesh_machine = o3d.io.read_triangle_mesh("./mesh/table_coro_tri.ply")
    o3d.visualization.draw_geometries([mesh_machine, artag])


if __name__ == '__main__':
    rospy.init_node('evaluation_maneuvrability', anonymous=True)
    arm = Doosan()
    # get_theorical_dust()
    o3d_tool = Open3dTool()
    data_m = DataManager()


    # Chemin vers le fichier contenant les JSON
    file_path = rospkg.RosPack().get_path('cleaning') + '/data/telemetrie/telemetrie_20230725-131603.json'

    # Extract point and guard relation
    result = {}
    actual_guard = tuple()

    # seqs = [220, 545, 745]
    # index = 0

    # next_mat = seqs[index]
    # actual_spot = spots[index]
    trans_mach2world = []
    now = rospy.get_rostime()
    theta_goal = 0
    with open(file_path, 'r') as file:
        for line in file:
            # Charger le JSON à partir de la ligne
            json_data = json.loads(line)
            if 'theta' in json_data:
                delta = rospy.get_rostime() - now
                now = rospy.get_rostime()
                print(delta)
                ros_message = convert_to_ros_message(line, 'geometry_msgs/Pose2D')

                trans_mach2world = get_trans_mat([ros_message.x, ros_message.y, ros_message.theta])
                # print(trans_mach2world)
            # Vérifier le type de message dans le JSON et convertir en conséquence
            elif 'position' in json_data:
                ros_message = convert_to_ros_message(line, 'sensor_msgs/JointState')
                actual_guard = tuple(ros_message.position)
                result.update({actual_guard: []})
                arm.go_to_j(list(actual_guard))
                while arm.check_motion() != 0:
                    rospy.sleep(0.1)
            elif 'pose' in json_data:
                ros_message = convert_to_ros_message(line, 'geometry_msgs/PoseStamped')
                if ros_message.header.frame_id == "success":
                    color = [0, 1, 0]
                    arm.go_to_l(ros_message.pose)
                    while arm.check_motion() != 0:
                        rospy.sleep(0.1)
                else:

                    color = [1, 0, 0]
                value = result.get(actual_guard)
                point = np.dot(trans_mach2world, np.array(
                    [ros_message.pose.position.x, ros_message.pose.position.y, ros_message.pose.position.z, 1]))
                # if index == 3:

                value.append([point[:3], color])
                result[actual_guard] = value
            else:
                # Gérer le cas où le type de message n'est pas reconnu
                print("Type de message non reconnu.")
                continue

    delta = rospy.get_rostime() - now
    now = rospy.get_rostime()
    print(delta)