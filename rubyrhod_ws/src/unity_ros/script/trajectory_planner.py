#!/usr/bin/env python3

from cmath import pi

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg

from utility.doosan import Doosan
from geometry_msgs.msg import PoseArray, Pose

'''
This script is a work-in-progress.
Once it work in an intended way, it will either have comments or will be transformed.
'''

def callback(msg):
    '''
    Receive a PoseArray and make the robot follow a trajectory
    '''
    arm = Doosan()
    display_trajectory_planner = \
        rospy.Publisher('/dsr01m1013/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

    ##########
    # Move J
    # for pose in msg.poses:
    #     rospy.loginfo(pose)

    #     plan = group.set_pose_target(pose)
    #     #plan = group.set_joint_value_target(pose, True)
    #     #plan = group.plan()
    #     plan = group.allow_replanning(True)

    #     plan = group.go(wait=True)

    #     while(arm.check_motion() != 0):
    #         rospy.sleep(1)

    #     group.stop()
    #     group.clear_pose_targets()

    ##########
    # Move L

    group = arm.move_group
    waypoints = []

    # First point need to be current position
    wpose = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))

    # Copy all the trajectory points in the waypoints' array
    for pose in msg.poses:
        waypoints.append(copy.deepcopy(pose))

    # Plan the trajectory
    (plan, fraction) = group.compute_cartesian_path(
        waypoints, 0.01, 0.0
    )

    # Execute planned path
    #group.execute(plan, wait=True)

    ##########
    # Joint Move
    # joint_goal = group.get_current_joint_values()

    # joint_goal[0] = 0
    # joint_goal[1] = -pi/4
    # joint_goal[3] = -pi/4
    # joint_goal[5] = pi/3

    # group.go(joint_goal, wait=True)

if __name__ == "__main__":
    '''
    Create the node and listen to a topic to get a PoseArray message
    '''

    rospy.init_node('unity_trajectory_node', anonymous=True)
    rospy.Subscriber('/vr/trajectory', PoseArray, callback, queue_size=1)

    rospy.spin()
