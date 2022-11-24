#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
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
    pub = rospy.Publisher("/dsr01m1013/grasp", Pose, queue_size=10)
    rospy.init_node('grasp_test', anonymous=True)

    rospy.Subscriber("grasp_result", Bool, callback_answers)

    # Test 1
    msg = Pose()
    msg.position.x = -0.07824018859863281
    msg.position.y = -0.4029244689941406
    msg.position.z = 0.638942382812
    msg.orientation.x =  0.331 #1.5358897417
    msg.orientation.y =  0.616 #-0.8377580409552000118
    msg.orientation.z = -0.673#-3.019419605942700091
    msg.orientation.w = -0.242#-3.019419605942700091
    pub.publish(msg)
    print("msg pub")
    rospy.spin()
