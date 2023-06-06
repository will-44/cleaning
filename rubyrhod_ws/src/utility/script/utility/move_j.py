#!/usr/bin/env python3

import rospy
from doosan import Doosan
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

''' This node get the joints position from a topic and send it to the doosan. 
 Only for testing '''
def callback(data):
    arm = Doosan()
    pub = rospy.Publisher('move_j_result', Bool, queue_size=10)

    pos = [data.position[0], data.position[1], data.position[2], data.position[3], data.position[4], data.position[5]]
    rospy.loginfo(pos)
    arm.go_to_j(pos)
    while( (arm.check_motion() != 0)):
        rospy.sleep(1)
    pub.publish(True)

if __name__ == '__main__':
    rospy.init_node('move_j', anonymous=True)
    rospy.Subscriber("move_j", JointState, callback)
    rospy.spin()


