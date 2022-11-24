#!/usr/bin/env python3

import rospy
from doosan import Doosan
from std_msgs.msg import Empty
from std_msgs.msg import Bool

'''
This node propose a topics to send the home position to the doosan
'''
def callback(data):
    arm = Doosan()
    pub = rospy.Publisher('home_result', Bool, queue_size=10)
    # The position of the home in rad
    pos = [1.519094383, 1.162374874, -1.94320531, 0.06055833159, 1.53860536, 0.079255698]
    arm.go_to_j(pos)
    while( (arm.check_motion() != 0)):
        rospy.sleep(1)
    pub.publish(True)

if __name__ == '__main__':
    rospy.init_node('home', anonymous=True)
    rospy.Subscriber("home", Empty, callback)
    rospy.spin()
