#!/usr/bin/env python3

import rospy
from doosan import Doosan
from robotiq import Robotiq
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import actionlib
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseAction, MoveBaseResult, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import tf

# Constant
PREEMPTED = 2

# Thoses variables are share in all methods
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

# Position of the objet in map frame
pose = PoseStamped() 
# Position of the object in arm frame
pose_arm = PoseStamped()

def move_base_client():
    '''
    This methode is call to start the action mir and them the action to grasp
    :return: the methodes callback_ active, done, feedback are call during the action
    '''

    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()
    goal = MoveBaseGoal()

    pose.header.stamp = rospy.get_rostime()
    goal.target_pose.header.frame_id = pose.header.frame_id
    goal.target_pose.header.stamp = pose.header.stamp
    goal.target_pose.pose.position.x = pose.pose.position.x
    goal.target_pose.pose.position.y = pose.pose.position.y
    goal.target_pose.pose.orientation.z = pose.pose.orientation.z
    goal.target_pose.pose.orientation.w = pose.pose.orientation.w

    client.send_goal(goal,
                     active_cb=callback_active,
                     feedback_cb=callback_feedback,
                     done_cb=callback_done)

    rospy.loginfo("Goal has been sent to the MiR action server.")

def callback_active():
    '''
    This methode is call at the beginning of the action
    '''

    rospy.loginfo("Action server is processing the goal")

def callback_done(state, result):
    '''
    This methode is call at the end of the action
    :param state: the state received are described in the msg files (_MoveBaseAction.py)
    :param result: the result received are described in the msg files (_MoveBaseAction.py) (appenrently it is nothing)
    :return: Send a message on the topics grasp_result. True if the action is done or False if some issues append
    '''

    # By default there is an error
    is_error = True
    pub = rospy.Publisher('grasp_complex_result', Bool, queue_size=10)
    # if the result is different to PREEMPTED (2) we don't try to grasp, it mean that something append bad
    if (state == PREEMPTED): 

        gripper = Robotiq()
        gripper.open()
        # TODO verifie if gPR is the right var to check
        while gripper.status().gPR != 0:
            rospy.sleep(1)
        # TODO verifie the quaternion position
        pos = [pose_arm.pose.position.x, pose_arm.pose.position.y, pose_arm.pose.position.z,
               pose_arm.pose.orientation.x, pose_arm.pose.orientation.y, pose_arm.pose.orientation.z]

        # Send the order to doosan and check the end of the movement
        is_error = not arm.go_to_l(pos)
        if not is_error:
            while arm.check_motion() != 0:
                rospy.sleep(1)

            # we need to check if the gripper is at the position desired and we close it.
            if arm.is_at_pos_asked(pos):
                is_error = False
                gripper.close()
            else:
                is_error = True

    # We publish in the topics to knows if the action succeed or not, that's why we inverse is_error
    pub.publish(not is_error)
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))


def callback_feedback(feedback):
    global pose_arm

    rospy.loginfo("on prend le feedback")

    # transform the pose from the map to the basefootprint mir
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        trans_mir = tf_buffer.lookup_transform('base_footprint', 'map', rospy.Time(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("pb dans la transformation")
        return

    pose_mir = tf2_geometry_msgs.do_transform_pose(pose, trans_mir)
    rospy.loginfo(pose_mir)  #TODO explicite var
    try:
        trans_arm = tf_buffer.lookup_transform('base_0', 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("pb dans la transformation")
        return
    pose_arm = tf2_geometry_msgs.do_transform_pose(pose_mir, trans_arm)
    rospy.loginfo(pose_arm)

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

    #  Check if the point is reachable and stop the mir, that start the grasp with the doosan
    if(arm.is_pt_reachable(pose_arm.pose.position.x, pose_arm.pose.position.y, pose_arm.pose.position.z,
                           roll, pitch, yaw)):
        pose_arm.pose.orientation.x = roll
        pose_arm.pose.orientation.y = pitch
        pose_arm.pose.orientation.z = yaw
        client.cancel_all_goals()
        rospy.loginfo("The object is close enought, mir action done, arm get object")

def callback(data):
    '''

    :param data: The position of the object geometry_msgs.msg.Pose, using quaternions
    :return:
    '''
    pose.header.frame_id = "map"
    pose.pose.position.x = data.position.x
    pose.pose.position.y = data.position.y
    pose.pose.position.z = data.position.z
    pose.pose.orientation.x = data.orientation.x
    pose.pose.orientation.y = data.orientation.y
    pose.pose.orientation.z = data.orientation.z
    pose.pose.orientation.w = data.orientation.w
    try:
        move_base_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error in the move_base action")

if __name__ == '__main__':
    global arm
    rospy.init_node('grasp_node', anonymous=True)
    rospy.Subscriber("/dsr01m1013/grasp_complex", Pose, callback)
    arm = Doosan()
    # just activate the gripper
    gripper = Robotiq()
    gripper.reset()
    rospy.sleep(2)
    gripper.activate()
    rospy.sleep(5)

    rospy.spin()



