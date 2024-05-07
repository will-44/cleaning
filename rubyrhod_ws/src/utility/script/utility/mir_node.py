#! /usr/bin/env python3
import move_base_msgs.msg
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseAction, MoveBaseResult, MoveBaseGoal
from doosan import Doosan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import tf2_ros
import tf
import tf2_geometry_msgs

'''
This node wait for position ask and call the action to move the mir base
'''

client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
# Object pose un map frame
pose = PoseStamped()
pub = rospy.Publisher('/mir_result', Bool, queue_size=10)

def move_base_client():
    '''
    The user calls this method to start the action
    global goal: The desired position x, y, theta (quaternion)
    :return: the methodes callback_ active, done, feedback are call during the action
    '''
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = pose.header.frame_id
    goal.target_pose.header.stamp = rospy.get_rostime()
    pose.header.stamp = rospy.get_rostime()

    goal.target_pose.pose.position.x = pose.pose.position.x
    goal.target_pose.pose.position.y = pose.pose.position.y
    goal.target_pose.pose.orientation.x = pose.pose.orientation.x
    goal.target_pose.pose.orientation.y = pose.pose.orientation.y
    goal.target_pose.pose.orientation.z = pose.pose.orientation.z
    goal.target_pose.pose.orientation.w = pose.pose.orientation.w
    rospy.loginfo(goal)
    client.send_goal(goal,
                     active_cb=callback_active,
                     feedback_cb=callback_feedback,
                     done_cb=callback_done)

    rospy.loginfo("Goal has been sent to the action server.")


def callback_active():
    '''
    This method is called when the action is call
    :return:
    '''
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    '''
    this method is called when the action is done
    :param state: the state is succeed (3), aborted or preented, described in the msg files (_MoveBaseAction.py)
    :param result: there is not result for this action
    :return: Send a message on the topics mir_result. True if the action is done or False if some issues append
    '''
    if state == 3:
        pub.publish(True)
    else:
        pub.publish(False)
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))


def callback_feedback(feedback):
    '''
    This method is called during the execution
    :param feedback: It give the actual position of the mir
    :return:
    '''
    # If we need feedback, ask it here
    rospy.loginfo("Feedback:%s" % str(feedback))
    return

def callback(pose_desired):
    '''
    Get the posiiton from PoseStamped transform and call the mir action (the position z always equal 0)
    :param pose_desired:
    :return:
    '''
    # quaternion = tf.transformations.quaternion_from_euler(0, 0, pose_desired.theta)
    if pose_desired.header.frame_id is None:
        pose.header.frame_id = "map"
    else:
        pose.header.frame_id = pose_desired.header.frame_id
    pose.header.stamp = rospy.get_rostime()
    pose.pose.position.x = pose_desired.pose.position.x
    pose.pose.position.y = pose_desired.pose.position.y
    pose.pose.position.z = 0
    pose.pose.orientation.x = pose_desired.pose.orientation.x
    pose.pose.orientation.y = pose_desired.pose.orientation.y
    pose.pose.orientation.z = pose_desired.pose.orientation.z
    pose.pose.orientation.w = pose_desired.pose.orientation.w

    move_base_client()


if __name__ == '__main__':
    rospy.init_node('mir_ac')
    rospy.Subscriber("/mir_go_to", PoseStamped, callback)
    rospy.spin()
