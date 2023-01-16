#!/usr/bin/env python3

import rospy
from doosan import Doosan
from robotiq import Robotiq
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker
import rviz_tool
import tf

from utility.srv import GraspVacuum, GraspVacuumResponse


def handle_grasp_vacuum(req):
    global arm, gripper

    # Open gripper
    gripper.open()

    # wait the gripper open
    # while gripper.status().gPR != 0:
    #     rospy.sleep(0.5)

    # Move arm to entry point
    res = arm.go_to_j([0, 0, 1.57, 0, 0, 0])

    # Move arm to vacuum
    if res:
        res = arm.go_to_j([0, 0, 1.57, 0, 1.57, 0])

    # Close gripper
    if res:
        gripper.close()

    # wait the gripper close
    # while gripper.status().gPR != 0:
    #     rospy.sleep(0.5)

    # Pull the vacuum
    if res:
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.5
        pose.position.z = 0.5
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        res = arm.go_to_l(pose)

    answer = GraspVacuumResponse()
    answer.result = res
    return answer







if __name__ == "__main__":
    global arm, gripper
    rospy.init_node('grasp_vacuum_server')
    s = rospy.Service('grasp_vacuum', GraspVacuum, handle_grasp_vacuum)
    gripper = Robotiq()
    arm = Doosan()

    print("Ready to take vacuum")
    rospy.spin()
