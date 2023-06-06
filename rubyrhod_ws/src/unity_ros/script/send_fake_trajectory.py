#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
'''
This is a test script, it is only use to send data to ROS to simulate data from the VR.
Remove later when not needed.
'''
def pub_trajectory():
    pub = rospy.Publisher('/vr/trajectory/', PoseArray, queue_size=1)
    rospy.init_node('unity_trajectory_test', anonymous=True)
    
    pose_1 = Pose()
    pose_1.position.x = .4
    pose_1.position.y = .4
    pose_1.position.z = 1
    pose_1.orientation.w = 1
    
    pose_2 = Pose()
    pose_2.position.x = .4
    pose_2.position.y = .5
    pose_2.position.z = 1
    pose_2.orientation.w = 1
    
    pose_3 = Pose()
    pose_3.position.x = .5
    pose_3.position.y = .4
    pose_3.position.z = 1
    pose_3.orientation.w = 1
    
    pose_4 = Pose()
    pose_4.position.x = .4
    pose_4.position.y = .4
    pose_4.position.z = .9
    pose_4.orientation.w = 1
    
    trajectory = PoseArray(poses=(pose_1,pose_2,pose_3,pose_4))
    rospy.loginfo(trajectory)
    pub.publish(trajectory)
    
if __name__ == '__main__':
    try:
        pub_trajectory()
    except rospy.ROSInterruptException:
        pass