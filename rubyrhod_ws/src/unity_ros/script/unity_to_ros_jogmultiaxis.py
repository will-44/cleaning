#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64MultiArray
from dsr_msgs.msg import JogMultiAxis

'''
This script is used to tranform a Float64MultiArray message from Unity
to a JobMultiAxis that will then be send to the Doosan arm.
'''

# Global publisher to create once
pub = rospy.Publisher('/dsr01m1013/jog_multi', JogMultiAxis, queue_size=1)

def callback(msg):
    '''
    Receive a message and transform it into a JobMultiAxis before sending it to the Doosan arm.
    :param msg: the linear and angular velocity. dsr_msgs.JogMultiAxis
    '''

    # Initialize the message
    new_jogmultiaxis = JogMultiAxis()
    # Check JogMultiAxis documentation to understand the value bellow.
    new_jogmultiaxis.jog_axis = msg.data 
    new_jogmultiaxis.move_reference = 0
    new_jogmultiaxis.speed = 100

    rospy.loginfo(new_jogmultiaxis)

    pub.publish(new_jogmultiaxis)

def convert_float64multiarray_to_jogmultiaxis():
    '''
    Create the node and listen to a topic to get a Float64MultiArray message
    '''

    rospy.init_node('unity_float64multiarrat_to_jobmultiaxis')
    rospy.Subscriber('/unity/jogmultiaxis', Float64MultiArray, callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    try:
        convert_float64multiarray_to_jogmultiaxis()
    except rospy.ROSInterruptException:
        pass