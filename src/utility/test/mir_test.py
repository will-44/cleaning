#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

'''
This test send a position to the node grasp to grasp an object
and listen the result of the operation
'''
def callback_answers(answers):
    rospy.loginfo("NB TEST: 1")
    if answers.data:
        rospy.loginfo("\033[1;32mTEST: PASSED")
    else:
        rospy.loginfo("\033[1;31mTEST: FAILED")



if __name__ == '__main__':
    mir_pub = rospy.Publisher("mir_go_to", Pose2D, queue_size=10)
    rospy.init_node('mir_test', anonymous=True)

    rospy.Subscriber("mir_result", Bool, callback_answers)
    rate = rospy.Rate(0.5)
    # while not rospy.is_shutdown():
        # Test 1
    msg = Pose2D()
    msg.x = 12.881
    msg.y = 25.023
    msg.theta = 1.53588974
    print(msg)

    mir_pub.publish(msg)
        # rate.sleep()
    rospy.spin()
