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
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2
import open3d_ros_helper as o3d_ros


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


def callback_mir(msg):
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


def callback_pcd(msg):
    global dict_pos2pts, broadcaster, pcl_pub, j1, j2, j3, j4, j5, j6

    # Add points to dict
    pcd = o3d_ros.rospc_to_o3dpc(msg)
    dict_pos2pts[(j1, j2, j3, j4, j5, j6)] = np.asarray(pcd.points)

    joints, all_poses_check = increase_joint()
    if all_poses_check:
        return

    matrix, is_valid, tcp_machine = get_cone_translation(joints)
    while not is_valid:
        # add Joints
        joints, all_poses_check = increase_joint()
        if all_poses_check:
            return
        matrix, is_valid, tcp_machine = get_cone_translation(joints)

    # send tf
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "machine"
    static_transformStamped.child_frame_id = "cone"
    static_transformStamped.transform.translation.x = tcp_machine.pose.position.x
    static_transformStamped.transform.translation.y = tcp_machine.pose.position.y
    static_transformStamped.transform.translation.z = tcp_machine.pose.position.z
    static_transformStamped.transform.rotation.x = tcp_machine.pose.orientation.x
    static_transformStamped.transform.rotation.y = tcp_machine.pose.orientation.y
    static_transformStamped.transform.rotation.z = tcp_machine.pose.orientation.z
    static_transformStamped.transform.rotation.w = tcp_machine.pose.orientation.w

    broadcaster.sendTransform(static_transformStamped)

    # send matrix to node
    msg = Float64MultiArray()
    msg.data = matrix
    msg.layout.dim[0].label = "height"
    msg.layout.dim[0].size = 4
    msg.layout.dim[0].stride = 16
    msg.layout.dim[1].label = "width"
    msg.layout.dim[1].size = 4
    msg.layout.dim[1].stride = 4

    pcl_pub.publish(msg)


def increase_joint():
    global j1, j2, j3, j4, j5, j6
    all_poses_check = False
    # Add x degree to the joints
    j6 = 0
    j5 += 10
    if j5 > 90:
        j5 = -90
        j4 += 10
        if j4 > 180:
            j4 = -180
            j3 += 10
            if j3 > 90:
                j3 = -90
                j2 += 10
                if j2 > 90:
                    j2 = -90
                    j1 += 10
                    if j1 > -30:
                        j1 = -150
                        all_poses_check = True
    print(j1, j2, j3, j4, j5, j6)
    return [radians(j1), radians(j2), radians(j3),
            radians(j4), radians(j5), radians(j6)], all_poses_check


def get_cone_translation(pose):
    '''
    Return the TCP pose in the map frame
    :param pose:
    :return: translatio matrix  and is pose valide
    '''
    global mir_result, arm, broadcaster

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

            return cone_transform, True, tcp_machine
    return [0, 0, 0], False, [0, 0, 0]


if __name__ == '__main__':
    global mir_result, arm, broadcaster, j1, j2, j3, j4, j5, j6, dict_pos2pts, pcl_pub
    rospy.init_node('moveit_pcd', anonymous=True)
    arm = Doosan()
    mir_result = False

    # Subscribers
    rospy.Subscriber("/mir_result", Bool, callback_mir)
    rospy.Subscriber("/pcl_result", PointCloud2, callback_pcd)

    mir_pub = rospy.Publisher('/mir_go_to', PoseStamped, queue_size=10)
    pcl_pub = rospy.Publisher('/transf_mat', Float64MultiArray, queue_size=10)

    # Dictionnary
    dict_pos2pts = {}

    # Joint poses
    j1 = -150
    j2 = -90
    j3 = -90
    j4 = -180
    j5 = -90
    j6 = 0

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

    joints = [radians(j1), radians(j2), radians(j3),
              radians(j4), radians(j5), radians(j6)]
    matrix, is_valid, tcp_machine = get_cone_translation(joints)
    while is_valid != True:
        # increase Joints angles
        # print(is_valid)
        joints, all_poses_check = increase_joint()

        if not all_poses_check:
            matrix, is_valid, tcp_machine = get_cone_translation(joints)
        else:
            break
    print("all_pose", all_poses_check)
    if not all_poses_check:
        # send tf
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "machine"
        static_transformStamped.child_frame_id = "cone"
        static_transformStamped.transform.translation.x = tcp_machine.pose.position.x
        static_transformStamped.transform.translation.y = tcp_machine.pose.position.y
        static_transformStamped.transform.translation.z = tcp_machine.pose.position.z
        static_transformStamped.transform.rotation.x = tcp_machine.pose.orientation.x
        static_transformStamped.transform.rotation.y = tcp_machine.pose.orientation.y
        static_transformStamped.transform.rotation.z = tcp_machine.pose.orientation.z
        static_transformStamped.transform.rotation.w = tcp_machine.pose.orientation.w

        broadcaster.sendTransform(static_transformStamped)

        # send matrix to node
        msg = Float64MultiArray()
        msg.data = matrix
        msg.layout.dim[0].label = "height"
        msg.layout.dim[0].size = 4
        msg.layout.dim[0].stride = 16
        msg.layout.dim[1].label = "width"
        msg.layout.dim[1].size = 4
        msg.layout.dim[1].stride = 4

        pcl_pub.publish(msg)
        rospy.loginfo("msg send")
    rospy.loginfo("no msg send")

    rospy.spin()
    # with alive_bar(2519424) as bar:
    #                         bar()
