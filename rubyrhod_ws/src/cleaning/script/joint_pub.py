#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint



rospy.init_node("franka_joint_command", anonymous=True)


joint_command_publisher = rospy.Publisher('/joint_command', JointTrajectory, queue_size=10)

trajectory_msg = JointTrajectory()
trajectory_msg.joint_names = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7'
]

# Première position
point1 = JointTrajectoryPoint()
point1.positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
point1.time_from_start = rospy.Duration(1.0)  # Temps en secondes

# Deuxième position
point2 = JointTrajectoryPoint()
point2.positions = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
point2.time_from_start = rospy.Duration(2.0)  # Temps en secondes

# Troisième position
point3 = JointTrajectoryPoint()
point3.positions = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
point3.time_from_start = rospy.Duration(3.0)  # Temps en secondes

trajectory_msg.points = [point1, point2, point3]


joint_command_publisher.publish(trajectory_msg)
