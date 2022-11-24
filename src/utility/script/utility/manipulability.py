#!/usr/bin/env python3

import rospy
from doosan import Doosan
from robotiq import Robotiq
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose2D
from visualization_msgs.msg import Marker
import rviz_tool
import tf


if __name__ == '__main__':
    rospy.init_node('manipulability_node', anonymous=True)
    mir_pub = rospy.Publisher("mir_go_to", Pose2D, queue_size=10)
    arm = Doosan()
