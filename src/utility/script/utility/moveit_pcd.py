#!/usr/bin/env python3

import rospy
import rospkg
from doosan import Doosan
from robotiq import Robotiq
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose2D
from visualization_msgs.msg import Marker
from rviz_tool import display_marker
import tf
import geometry_msgs.msg
import pickle
import open3d as o3d
import numpy as np
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from math import radians, degrees
import copy

from alive_progress import alive_bar



from tf.transformations import euler_from_quaternion, quaternion_from_euler, \
    quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


def PoseStamped_2_mat(p):
    q = p.orientation
    pos = p.position
    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = np.array([pos.x, pos.y, pos.z])
    return T


def check_collision(poses):
    """
    :param poses: joints poses in rad
    :return:
    """
    try:
        rospy.wait_for_service('/dsr01m1013/check_state_validity', timeout=1)
    except:
        return False

    try:
        robot = RobotState()
        robot.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        robot.joint_state.position = poses
        check_srv = rospy.ServiceProxy('/dsr01m1013/check_state_validity', GetStateValidity)
        result = check_srv(group_name='arm', robot_state=robot)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: s")


def fkin(poses):
    """

    :param pos: position in degree
    :param ref: default world 0
    :return: pos in metre and radian

    """
    try:
        rospy.wait_for_service('/dsr01m1013/compute_fk', timeout=5)
    except:
        return False
    try:
        robot = RobotState()
        robot.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        robot.joint_state.position = poses
        msg = GetPositionFK()
        fkin_srv = rospy.ServiceProxy('/dsr01m1013/compute_fk', GetPositionFK)
        result = fkin_srv(fk_link_names=['link6'], robot_state=robot)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: s")


if __name__ == '__main__':
    rospy.init_node('moveit_pcd', anonymous=True)
    arm = Doosan()

    mir_pub = rospy.Publisher('mir_go_to', PoseStamped, queue_size=1)

    pcd_path = rospkg.RosPack().get_path('utility') + "/mesh/scie1.ply"
    cone_path = rospkg.RosPack().get_path('utility') + "/mesh/cone.stl"

    pcd_load = o3d.io.read_point_cloud(pcd_path)
    cone_load = o3d.io.read_triangle_mesh(cone_path)
    cone_load = cone_load.sample_points_poisson_disk(3000)
    spot_path = rospkg.RosPack().get_path('utility') + "/mesh/spot.ply"
    spots = o3d.io.read_point_cloud(spot_path)
    spots_np = np.asarray(spots.points)
    # spot_path = rospkg.RosPack().get_path('utility') + "/data/best_spot.pkl"
    # with open(spot_path, 'rb') as f:
    #     best_spot = pickle.load(f)
    print("center pcd")
    print(pcd_load.get_center())

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()

    mesh_path = rospkg.RosPack().get_path('utility') + "/mesh/scie1.obj"
    machine_pose = geometry_msgs.msg.PoseStamped()
    machine_pose.header.frame_id = 'map'
    machine_pose.pose.position.x = 16.6
    machine_pose.pose.position.y = 3.6
    machine_pose.pose.position.z = -0.2
    machine_pose.pose.orientation.x = 0.707
    machine_pose.pose.orientation.y = 0#-0.707
    machine_pose.pose.orientation.z = 0#-0.707
    machine_pose.pose.orientation.w = 0.707
    arm.scene.add_mesh("machine", machine_pose, mesh_path)

    # base = copy.deepcopy(cone_load)
    # base.translate(((best_spot[0][0] - pcd_load.get_center()[0]), (best_spot[0][1] - pcd_load.get_center()[1]), 1.3))
    # R = base.get_rotation_matrix_from_xyz((0, 0, np.pi))
    # R = cone.get_rotation_matrix_from_quaternion([machine_pose.pose.orientation.x, machine_pose.pose.orientation.y,
    #                                               machine_pose.pose.orientation.z, machine_pose.pose.orientation.w])
    # base.rotate(R)
    #
    # pose = [radians(0), radians(0), radians(0),
    #         radians(0), radians(0), radians(0)]
    # trans = fkin(pose).pose_stamped[0].pose
    # link6 = copy.deepcopy(base)
    # link6.translate((-trans.position.x, -trans.position.y, trans.position.z))

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

    # vis.add_geometry(pcd_load)
    # vis.add_geometry(base)
    # vis.add_geometry(mesh)
    # new_cone = copy.deepcopy(link6)
    # vis.add_geometry(new_cone)
    # vis.update_geometry(new_cone)
    # vis.poll_events()
    # vis.update_renderer()

    rospy.loginfo(spots_np)
    display_marker(Marker.CUBE, spots_np[0][0], spots_np[0][1], spots_np[0][2], 0, 0, 0, 1, "machine")


    # with alive_bar(2519424) as bar:
    #     for teta1 in range(-150, -30, 10):
    #         for teta2 in range(-90, 90, 10):
    #             for teta3 in range(-90, 90, 10):
    #                 for teta4 in range(-180, 180, 10):
    #                     for teta5 in range(-90, 90, 10):
    #                         teta6 = 0
    #
    #                         pose = [radians(teta1), radians(teta2), radians(teta3),
    #                                 radians(teta4), radians(teta5), radians(teta6)]
    #                         trans = fkin(pose).pose_stamped[0].pose
    #                         if radians(160) >= trans.orientation.z >= radians(30) and \
    #                                 radians(160) >= trans.orientation.y >= radians(20):
    #                             # print(degrees(trans.orientation.x))
    #                             # print(degrees(trans.orientation.y))
    #                             # while(arm.check_motion() != 0):
    #                             #     rospy.sleep(0.1)
    #
    #                             if check_collision(pose).valid:
    #                                 # vis.remove_geometry(new_cone)
    #                                 new_cone = copy.deepcopy(base)
    #                                 # trans = fkin(pose).pose_stamped[0].pose
    #                                 new_cone.translate((-trans.position.x, -trans.position.y, trans.position.z))
    #                                 R = new_cone.get_rotation_matrix_from_quaternion([trans.orientation.x,
    #                                                                                   trans.orientation.y,
    #                                                                                   trans.orientation.z,
    #                                                                                   trans.orientation.w])
    #                                 new_cone.rotate(R)
    #                                 R = new_cone.get_rotation_matrix_from_xyz([0, 0, np.pi])
    #                                 new_cone.rotate(R)
    #                                 o3d.io.write_point_cloud('/home/guillaume/Documents/cones/' + str(pose) + '.ply', new_cone)
    #                                 input()
    #
    #                                 # vis.add_geometry(new_cone)
    #                                 # vis.update_geometry(new_cone)
    #                                 # vis.poll_events()
    #                                 # vis.update_renderer()
    #                         bar()
    #                                 # o3d.visualization.draw_geometries([new_cone, pcd_load, base, mesh])
    #                                 # input()
