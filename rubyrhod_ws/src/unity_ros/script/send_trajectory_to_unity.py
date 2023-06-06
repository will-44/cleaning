#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory

"""
This script take a DisplayTrajectory message from Moveit and send a JointTrajectory
to ROS.
"""

# Global publisher to create once
pub = rospy.Publisher('/unity/trajectory/planned', JointTrajectory, queue_size=1)

def pub_planned_trajectory(msg):
    '''
    Publish to ROS the JointTrajectory message part of the DisplayTrajectory message
    to use non-Moveit specific message in ROS.
    '''

    rospy.loginfo('{nodeName} : Trajectory points are being publish'.format(nodeName = rospy.get_name()))
    
    pub.publish(msg.trajectory[0].joint_trajectory)

def sub_planned_trajectory():
    '''
    Subscribe to moveit to get the DisplayTrajectory message from Moveit
    '''
    
    rospy.init_node('unity_planned_trajectory_node')
    rospy.Subscriber('dsr01m1013/move_group/display_planned_path', DisplayTrajectory, pub_planned_trajectory, queue_size=1)

    rospy.sleep(.5)
    rospy.spin()


if __name__ == "__main__":
    try:
        sub_planned_trajectory()
    except rospy.ROSInterruptException:
        pass