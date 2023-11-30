#!/usr/bin/env python3

import roslib
from geometry_msgs.msg import Pose, Point, Quaternion

roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker, MarkerArray
import rospy

publisher = rospy.Publisher("visualization_marker", Marker, queue_size=100)
array_publisher = rospy.Publisher("visualization_markers", MarkerArray, queue_size=100)


def display_marker(marker_type: object, x: object, y: object, z: object, t: object, u: object, v: object, w: object,
                   ref: object) -> object:
    '''
    Create a marker on Rviz
    :param marker_type: the type of marker (arrow, sphere...)
    :param x: pos x
    :param y: pos y
    :param z: pos z
    :param t: orientation  x (quaternion)
    :param u: orientation  y (quaternion)
    :param v: orientation  z (quaternion)
    :param w: orientation  w (quaternion)
    :param ref: Which referential is it linked
    :return:
    '''

    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = ref
    m.header.stamp = rospy.Time.now()
    m.ns = 'marker_test_%d' % marker_type
    m.id = 0
    m.type = marker_type
    m.pose.orientation.x = t
    m.pose.orientation.y = u
    m.pose.orientation.z = v
    m.pose.orientation.w = w

    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z

    if marker_type == Marker.ARROW:
        m.scale.x = 0.5
        m.scale.y = 0.05
        m.scale.z = 0.05
    else:
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.scale.z = 0.05
    m.color.a = 1
    m.color.r = 1
    m.color.g = 0
    m.color.b = 0
    publisher.publish(m)


def display_marker_array(marker_type, poses, ref):
    """
    Send many markers to rviz
    :param marker_type: type
    :param poses: position and orientation
    :param ref: Tf
    :return:
    """
    msg = MarkerArray()
    all_marks = []
    i = 0
    for i, pose in enumerate(poses):
        m = Marker()
        m.action = Marker.ADD
        m.header.frame_id = ref
        m.header.stamp = rospy.Time.now()
        m.ns = 'marker_test_%d' % i
        # i += 1
        m.id = 0
        m.type = marker_type
        pos = Pose(Point(pose[0], pose[1], pose[2]), Quaternion(0, 0, 0, 0))
        m.pose = pos

        if marker_type == Marker.ARROW:
            m.scale.x = 0.5
            m.scale.y = 0.05
            m.scale.z = 0.05
        else:
            m.scale.x = 0.01
            m.scale.y = 0.01
            m.scale.z = 0.01
        m.color.a = 1

        m.color.r = 0  # colors[i][0]
        m.color.g = 0  # colors[i][1]
        m.color.b = 1  # colors[i][2]
        all_marks.append(m)
    msg.markers = all_marks
    array_publisher.publish(msg)


def display_text_array(points, ref):
    '''
    this function send a marker msg to rviz to
    display text at each points positions
    :param points: PoseStamped array
    :return:
    '''
    msg = MarkerArray()
    all_marks = []
    for i, point in enumerate(points):
        m = Marker()
        m.action = Marker.ADD
        m.header.frame_id = ref
        m.header.stamp = rospy.Time.now()
        m.ns = 'marker_test_%d' % i
        # i += 1
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.text = str(i)

        m.pose = point.pose

        m.scale.z = 0.1
        m.color.a = 1
        m.color.r = 1
        m.color.g = 0
        m.color.b = 0
        all_marks.append(m)
    msg.markers = all_marks
    array_publisher.publish(msg)

def reset_rviz():
    m = Marker()
    m.action = 3
    msg = MarkerArray()
    msg.markers = [m]
    array_publisher.publish(msg)
