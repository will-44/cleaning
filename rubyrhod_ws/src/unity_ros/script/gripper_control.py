#!/usr/bin/env python3

import rospy

# from utility.robotiq import Robotiq
from utility.robotiq import Robotiq
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from std_msgs.msg import Float32

"""
This script is used to control the gripper's position from a Unity message
"""
gripper_pub = rospy.Publisher('/gripper/unity/position', Float32, queue_size=1)

def callback(msg):
    """
    Use the message data to determine the position to send to the gripper

    :param msg: ROS message of type Float32
    """
    # Determine the desired position
    desiredPos = msg.data / 100 * gripper.openPosition
    # Send the command to the gripper
    gripper.custom_order(desiredPos)

def position_status(msg):
    """
    Publish the position of the gripper on ROS

    :param msg: ROS message of type Float32
    """
    gripper_pub.publish(msg.position)


def convert_unity_message():
    '''
    Create the node and listen to a topic to change the type of message use
    for the gripper.
    '''
    rospy.init_node('unity_gripper_node')
    rospy.Subscriber('/unity/gripper/pos', Float32, callback, queue_size=1)
    rospy.Subscriber('/gripper/stat', GripperStat, position_status, queue_size=1)

    global gripper 
    gripper = Robotiq()

    rospy.spin()

if __name__ == '__main__':
    try:
        convert_unity_message()
    except rospy.ROSInterruptException:
        pass