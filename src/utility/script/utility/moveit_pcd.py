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
from math import radians, degrees, atan2
import copy

from alive_progress import alive_bar

from tf.transformations import euler_from_quaternion, quaternion_from_euler, \
    quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2
from open3d_ros_helper import open3d_ros_helper as o3d_ros



def callback_mir(msg):
    global mir_result
    print("mir_result:", msg)
    mir_result = msg





def move_base(x, y, theta):
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
    msg.layout.dim.append(MultiArrayDimension())
    msg.layout.dim.append(MultiArrayDimension())
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

    tcp_map = arm.fkin(pose).pose_stamped[0]
    quaternion = (
        tcp_map.pose.orientation.x,
        tcp_map.pose.orientation.y,
        tcp_map.pose.orientation.z,
        tcp_map.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    if radians(160) >= euler[2] >= radians(30) and \
            radians(160) >= euler[1] >= radians(20):

        check = arm.check_collision(pose)
        if check.valid:
            # print("coucou")
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
            cone_transform.resize(1, 16)
            print(cone_transform[0])
            return cone_transform[0], True, tcp_machine
    return [0, 0, 0], False, [0, 0, 0]

def find_mir_angle(pose2D, pcd_associate):
    """ TEST
    Get the 2d pose and return the orientation referentiel of the machine
    :param pose2D:
    :param pcd_associate: open3D pcd
    :return: orientation in radian
    """
    # get 2d pose of all pcd points
    pcd_associate_np  = np.asarray(pcd_associate.points)
    # mean thoses poses x, y
    x = pcd_associate_np[:, 0]
    y = pcd_associate_np[:, 1]
    x_mean = np.mean(x)
    y_mean = np.mean(y)

    # get the orientation 2D with the atan
    a = pose2D[0] - x_mean
    b = pose2D[1] - y_mean

    angle = atan2(a, b)

    return angle



if __name__ == '__main__':
    global mir_result, arm, broadcaster, j1, j2, j3, j4, j5, j6, dict_pos2pts, pcl_pub
    # INIT
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
    j2 = -60 #-90
    j3 = 10#-90
    j4 = -180 #-180
    j5 = 50 #-90
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

    arm.add_machine_colision( rospkg.RosPack().get_path('utility') + "/mesh/scie1.obj")

    rospy.loginfo(spots_np)
    display_marker(Marker.CUBE, spots_np[0][0], spots_np[0][1], spots_np[0][2], 0, 0, 0, 1, "machine")
    # END INIT

    # MOVE MIR
    move_base(spots_np[0][0], spots_np[0][1], 0)

    while not mir_result:
        rospy.sleep(0.1)
    print("mir finish")
    # END MIR MOVE

    #START ALGO
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose_valid = []
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    joints = [radians(j1), radians(j2), radians(j3),
              radians(j4), radians(j5), radians(j6)]
    matrix, is_valid, tcp_machine = get_cone_translation(joints)
    all_poses_check = False
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
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())
        # print(matrix)
        msg.data = list(matrix)
        msg.layout.dim[0].label = "height"
        msg.layout.dim[0].size = 4
        msg.layout.dim[0].stride = 4*4
        msg.layout.dim[1].label = "width"
        msg.layout.dim[1].size = 4
        msg.layout.dim[1].stride = 4

        pcl_pub.publish(msg)
        rospy.loginfo("msg send")
    rospy.loginfo("no msg send")

    rospy.spin()

