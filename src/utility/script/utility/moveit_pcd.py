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

def callback(msg):
    global mir_result
    print("mir_result:", msg)
    mir_result = msg

if __name__ == '__main__':
    global mir_result
    rospy.init_node('moveit_pcd', anonymous=True)
    arm = Doosan()
    mir_result = False
    rospy.Subscriber("/mir_result", Bool, callback)
    mir_pub = rospy.Publisher('/mir_go_to', PoseStamped, queue_size=10)

    arm.go_to_j([0, 0, 0, 0, 0, 0])
    while arm.check_motion() != 0:
        rospy.sleep(0.1)


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
    machine_pose.pose.orientation.y = 0  # -0.707
    machine_pose.pose.orientation.z = 0  # -0.707
    machine_pose.pose.orientation.w = 0.707
    arm.scene.add_mesh("machine", machine_pose, mesh_path)

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

    rospy.loginfo(spots_np)
    display_marker(Marker.CUBE, spots_np[0][0], spots_np[0][1], spots_np[0][2], 0, 0, 0, 1, "machine")
    pose_mir = PoseStamped()
    pose_mir.header.frame_id = "machine"
    pose_mir.pose.position.x = spots_np[0][0]
    pose_mir.pose.position.y = spots_np[0][1]
    pose_mir.pose.orientation.x = 0
    pose_mir.pose.orientation.y = 0
    pose_mir.pose.orientation.z = -0.707
    pose_mir.pose.orientation.w = 0.707

    mir_pub.publish(pose_mir)
    rospy.loginfo("pose send")
    while not mir_result:
        rospy.sleep(0.1)
    print("mir finish")
    pose_valid = []
    with alive_bar(2519424) as bar:
        for teta1 in range(-150, -30, 10):
            for teta2 in range(-90, 90, 10):
                for teta3 in range(-90, 90, 10):
                    for teta4 in range(-180, 180, 10):
                        for teta5 in range(-90, 90, 10):
                            teta6 = 0

                            pose = [radians(teta1), radians(teta2), radians(teta3),
                                    radians(teta4), radians(teta5), radians(teta6)]
                            trans = fkin(pose).pose_stamped[0].pose
                            quaternion = (
                                trans.orientation.x,
                                trans.orientation.y,
                                trans.orientation.z,
                                trans.orientation.w)
                            euler = tf.transformations.euler_from_quaternion(quaternion)
                            if radians(160) >= euler[2] >= radians(30) and \
                                    radians(160) >= euler[1] >= radians(20):
                                #     arm.go_to_l(trans)
                                #     while arm.check_motion() != 0:
                                #         rospy.sleep(0.1)

                                if check_collision(pose).valid:
                                    pose_valid.append(pose)
                            bar()

