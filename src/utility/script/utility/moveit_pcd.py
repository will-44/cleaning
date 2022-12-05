#!/usr/bin/env python3

import rospy
from ros_numpy import numpify
import rospkg
from doosan import Doosan
from robotiq import Robotiq
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose2D
from visualization_msgs.msg import Marker
from rviz_tool import display_marker
import tf2_ros
import tf
import tf2_geometry_msgs
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

def add_machine_colision():
    global arm
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

def move_base(x, y):
    pose_mir = PoseStamped()
    pose_mir.header.frame_id = "machine"
    pose_mir.pose.position.x = x
    pose_mir.pose.position.y = y
    # TODO modifie the orientation
    pose_mir.pose.orientation.x = 0
    pose_mir.pose.orientation.y = 0
    pose_mir.pose.orientation.z = -0.707
    pose_mir.pose.orientation.w = 0.707

    mir_pub.publish(pose_mir)
    rospy.loginfo("pose send")

def send_cone_pose(pose):
    '''
    Return the TCP pose in the map frame
    :param pose:
    :return: translatio  and is pose valide
    '''
    tcp_map = fkin(pose).pose_stamped[0]
    quaternion = (
        tcp_map.pose.orientation.x,
        tcp_map.pose.orientation.y,
        tcp_map.pose.orientation.z,
        tcp_map.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    if radians(160) >= euler[2] >= radians(30) and \
            radians(160) >= euler[1] >= radians(20):
        check = check_collision(pose)
        if check.valid:
            pose_valid.append(pose)
            # transform the pose from the map to the base_footprint
            try:
                trans_machine = tf_buffer.lookup_transform('machine', 'map', rospy.Time(),
                                                           rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.loginfo("pb dans la transformation")
            tcp_machine = tf2_geometry_msgs.do_transform_pose(tcp_map, trans_machine)
            transformer = tf.TransformerROS()
            # Get transform matrix
            cone_transform = transformer.fromTranslationRotation((tcp_machine.pose.position.x,
                                                                  tcp_machine.pose.position.y,
                                                                  tcp_machine.pose.position.z),
                                                                 (
                                                                     tcp_machine.pose.orientation.x,
                                                                     tcp_machine.pose.orientation.y,
                                                                     tcp_machine.pose.orientation.z,
                                                                     tcp_machine.pose.orientation.w))
            display_marker(Marker.CUBE,
                           tcp_machine.pose.position.x,
                           tcp_machine.pose.position.y,
                           tcp_machine.pose.position.z,
                           tcp_machine.pose.orientation.x,
                           tcp_machine.pose.orientation.y,
                           tcp_machine.pose.orientation.z,
                           tcp_machine.pose.orientation.w,
                           "machine")
            return tcp_machine, True
    return 0, False

if __name__ == '__main__':
    global mir_result, arm
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


    add_machine_colision()

    rospy.loginfo(spots_np)
    display_marker(Marker.CUBE, spots_np[0][0], spots_np[0][1], spots_np[0][2], 0, 0, 0, 1, "machine")

    move_base(spots_np[0][0], spots_np[0][1])


    while not mir_result:
        rospy.sleep(0.1)
    print("mir finish")
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose_valid = []
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "machine"
    static_transformStamped.child_frame_id = "cone"

    with alive_bar(2519424) as bar:
        for teta1 in range(-150, -30, 10):
            for teta2 in range(-90, 90, 10):
                for teta3 in range(-90, 90, 10):
                    for teta4 in range(-180, 180, 10):
                        for teta5 in range(-90, 90, 10):
                            teta6 = 0
                            pose = [radians(teta1), radians(teta2), radians(teta3),
                                    radians(teta4), radians(teta5), radians(teta6)]

                            tcp, is_valid = send_cone_pose(pose)
                            # rospy.loginfo(is_valid)
                            if is_valid:

                                static_transformStamped.transform.translation.x = tcp.pose.position.x
                                static_transformStamped.transform.translation.y = tcp.pose.position.y
                                static_transformStamped.transform.translation.z = tcp.pose.position.z

                                static_transformStamped.transform.rotation.x = tcp.pose.orientation.x
                                static_transformStamped.transform.rotation.y = tcp.pose.orientation.y
                                static_transformStamped.transform.rotation.z = tcp.pose.orientation.z
                                static_transformStamped.transform.rotation.w = tcp.pose.orientation.w

                                broadcaster.sendTransform(static_transformStamped)
                            bar()
