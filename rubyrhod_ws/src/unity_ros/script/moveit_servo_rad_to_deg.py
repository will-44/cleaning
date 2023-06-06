#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64MultiArray
from utility.doosan import Doosan

'''
Listen to a Float64MultiArray msg that represent the joint position of the robot
and use it to make a move_j with the robot arm.
'''

def callback(msg):
    '''
    Send a command to the robot to do a move_j to the desired position
    :param msg: the position of the joints of the robot, in degree. std_msgs.Float64MultiArray
    '''
    
    arm.go_to_j(list(msg.data)) # The array needs to be in a list.


def convert_rad_to_deg():
    '''
    Create the node and listen to a topic to get a Float64MultiArray message
    '''

    rospy.init_node('convert_rad_to_deg')
    global arm 
    arm = Doosan()

    rospy.Subscriber('/joint_group_position_controller/command', Float64MultiArray, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        convert_rad_to_deg()
    except rospy.ROSInterruptException:
        pass