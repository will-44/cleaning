#!/usr/bin/env python3

import rospy
from doosan import Doosan
from robotiq import Robotiq
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import actionlib
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseAction, MoveBaseResult, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
import tf
from find_spot import get_pose_for_mobile
from visualization_msgs.msg import Marker
import rviz_tool

# constante:
PREEMPTED = 2
REACH = 3

# Thoses variables are share in all methods
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
# Position of the objet in map frame
pose_object = PoseStamped()
# Position of the object in arm frame
pose_arm = PoseStamped()


def check_is_reachable(pose):
    '''
    This methode verify if the object is reachable from the mir position
    :param pose: the object pose from map frame
    :return: if the object is reachable (true/false) and the position of the object from the base doosan
    '''
    # transform the pose from the map to the base_footprint
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        trans_mir = tf_buffer.lookup_transform('base_footprint', 'map', rospy.Time(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("pb dans la transformation")
        return

    pose_mir = tf2_geometry_msgs.do_transform_pose(pose, trans_mir)
    rospy.loginfo(pose_mir)
    try:
        trans_arm = tf_buffer.lookup_transform('base_0', 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("pb dans la transformation")
        return

    pose_mir_arm = tf2_geometry_msgs.do_transform_pose(pose_mir, trans_arm)
    rospy.loginfo(pose_mir_arm)

    # transform from quaternion to euler (doosan use euler angles)
    quaternion = (
        pose_mir_arm.pose.orientation.x,
        pose_mir_arm.pose.orientation.y,
        pose_mir_arm.pose.orientation.z,
        pose_mir_arm.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    #  Check if the point is reachable and stop the mir, that start the grasp with the doosan
    if (arm.is_pt_reachable(pose_mir_arm.pose.position.x, pose_mir_arm.pose.position.y, pose_mir_arm.pose.position.z,
                            roll, pitch, yaw)):
        return True, pose_mir_arm
    else:
        return False, pose_mir_arm


def move_base_client():
    '''
    This methode is call to start the action mir and them the action to grasp
    :return: the methodes callback_ active, done, feedback are call during the action
    '''

    rospy.loginfo("Waiting for action server to come up...")
    print(client)
    client.wait_for_server()
    goal = MoveBaseGoal()

    # Get mir pos
    odom_basefoot = rospy.wait_for_message("/odom", Odometry, timeout=10)
    # Convert to map tf
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        trans = tf_buffer.lookup_transform('map', 'odom', rospy.Time(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("pb dans la transformation mir to map")
        return

    odom_map = tf2_geometry_msgs.do_transform_pose(odom_basefoot.pose, trans)

    # transform pose mir in euler
    quaternion = (
        odom_map.pose.orientation.x,
        odom_map.pose.orientation.y,
        odom_map.pose.orientation.z,
        odom_map.pose.orientation.w)
    euler_mir = tf.transformations.euler_from_quaternion(quaternion)

    # transform pose object in euler
    quaternion = (
        pose_object.pose.orientation.x,
        pose_object.pose.orientation.y,
        pose_object.pose.orientation.z,
        pose_object.pose.orientation.w)
    euler_object = tf.transformations.euler_from_quaternion(quaternion)

    # Get best spot to grasp
    result_spot = get_pose_for_mobile(Pose2D(x=odom_map.pose.position.x,
                                             y=odom_map.pose.position.y,
                                             theta=euler_mir[2]),
                                      Pose2D(x=pose_object.pose.position.x,
                                             y=pose_object.pose.position.y,
                                             theta=euler_object[2]))

    # Transform to quaternion
    quaternion = tf.transformations.quaternion_from_euler(0, 0, result_spot.theta)

    pose_object.header.stamp = rospy.get_rostime()
    goal.target_pose.header.frame_id = pose_object.header.frame_id
    goal.target_pose.header.stamp = pose_object.header.stamp
    goal.target_pose.pose.position.x = result_spot.x
    goal.target_pose.pose.position.y = result_spot.y
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    print(goal)
    client.send_goal(goal,
                     active_cb=callback_active,
                     feedback_cb=callback_feedback,
                     done_cb=callback_done)

    rospy.loginfo("Goal has been sent to the MiR action server.")


def callback_active():
    '''
    This methode is call at the beginning of the action, it verify if the object is reachable without moving the mir
    '''
    global pose_arm

    result, pose_arm = check_is_reachable(pose_object)
    if result:
        client.cancel_all_goals()
        rospy.loginfo("The object is close enought, mir action done, arm get object")
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    '''
    This methode is call at the end of the action
    :param state: the state received are described in the msg files (_MoveBaseAction.py)
    :param result: the result received are described in the msg files (_MoveBaseAction.py)
    :return: Send a message on the topics grasp_result. True if the action is done or False if some issues append
    '''
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))
    # By default there is an error
    is_error = True
    pub = rospy.Publisher('grasp_complex_result', Bool, queue_size=10)
    res, pose_arm = check_is_reachable(pose_object)

    # if the result is different to REACH or PREEMPTED (2) we don't try to grasp, it mean that something append bad
    if state == REACH or state == PREEMPTED:
        gripper = Robotiq()
        gripper.open()
        while gripper.status().gPR != 0:
            rospy.sleep(1)

        # transform from quaternion to euler (doosan use euler angles)
        quaternion = (
            pose_arm.pose.orientation.x,
            pose_arm.pose.orientation.y,
            pose_arm.pose.orientation.z,
            pose_arm.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        rviz_tool.display_marker(Marker.ARROW, pose_arm.pose.position.x,
                                 pose_arm.pose.position.y, pose_arm.pose.position.z,
                                 pose_arm.pose.orientation.x, pose_arm.pose.orientation.y,
                                 pose_arm.pose.orientation.z, pose_arm.pose.orientation.w, "base_0")
        # Doosan use the ZYZ euler convention for angles
        pos = [pose_arm.pose.position.x, pose_arm.pose.position.y, pose_arm.pose.position.z, yaw, pitch, yaw]

        # Send the order to doosan and check the end of the movement
        is_error = not arm.go_to_l(pose_arm)
        if not is_error:
            while arm.check_motion() != 0:
                print("on check toujours")
                rospy.sleep(1)

            # we need to check if the gripper is at the position desired and we close it.
            if arm.is_at_pos_asked(pos):
                print("on ferme")
                is_error = False
                gripper.close()
            else:
                is_error = True

    # We publish in the topics to knows if the action succeed or not, that's why we inverse is_error
    pub.publish(not is_error)


def callback_feedback(feedback):
    rospy.loginfo("on prend le feedback")


def callback(data):
    '''

    :param data: The position of the object geometry_msgs.msg.Pose, using quaternions
    :return:
    '''
    rospy.loginfo("msg recu")
    pose_object.header.frame_id = "map"
    pose_object.pose.position.x = data.position.x
    pose_object.pose.position.y = data.position.y
    pose_object.pose.position.z = data.position.z
    pose_object.pose.orientation.x = data.orientation.x
    pose_object.pose.orientation.y = data.orientation.y
    pose_object.pose.orientation.z = data.orientation.z
    pose_object.pose.orientation.w = data.orientation.w
    rviz_tool.display_marker(Marker.ARROW, pose_object.pose.position.x, pose_object.pose.position.y,
                             pose_object.pose.position.z,
                             pose_object.pose.orientation.x, pose_object.pose.orientation.y,
                             pose_object.pose.orientation.z, pose_object.pose.orientation.w, 'map')
    try:
        move_base_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error in the move_base action")


if __name__ == '__main__':
    global arm
    rospy.init_node('grasp_node', anonymous=True)
    rospy.Subscriber("/dsr01m1013/grasp_complex", Pose, callback)
    arm = Doosan()
    print("activate")
    # just activate the gripper
    gripper = Robotiq()
    gripper.reset()
    rospy.sleep(2)
    print("activate")
    gripper.activate()
    gripper.close()
    rospy.sleep(5)

    rospy.spin()
