#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, TwistStamped

'''
This script is used to tranform a Twist message from Unity
to a TwistStamped that will then be send to moveit_servo.
    Note: Moveit_servo doesn't seem to work without the timestamped which can't
          be initialized in Unity.
'''

# # Global publisher to create once
pub = rospy.Publisher('/dsr01m1013/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)

def callback(msg):
    '''
    Receive a message and transform it into a TwistStamped before sending it to moveit_servo.
    :param msg: the linear and angular velocity. geometry_msgs.Twist
    '''
    
    # Initialize the message
    new_twist = TwistStamped()
    new_twist.header.stamp.secs = rospy.get_rostime().secs # Get the ROS time
    new_twist.header.stamp.nsecs = rospy.get_rostime().nsecs # Get the ROS time decimal
    new_twist.twist = msg

    rospy.loginfo(new_twist)
 
    pub.publish(new_twist)

def convert_twist_to_twiststamped():
    '''
    Create the node and listen to a topic to get a Twist message
    '''

    rospy.init_node('unity_twist_to_twiststamped')
    rospy.Subscriber('/unity/twist', Twist, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        convert_twist_to_twiststamped()
    except rospy.ROSInterruptException:
        pass