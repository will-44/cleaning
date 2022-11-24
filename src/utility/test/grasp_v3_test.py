#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

'''
This test send a position to the node grasp_complexe to grasp an object far from the robot
and listen the result of the operation
'''
state = 1
is_occupied = False


def callback_answers(answers):
    global state, is_occupied
    expected = False
    if state == 0:
        expected = True
    elif state == 1:
        expected = False
    elif state == 2:
        expected = False

    rospy.loginfo("\033[1;37mTEST : " + str(state))

    if answers.data == expected:
        rospy.loginfo("\033[1;32mTEST: PASSED\033[1;37m")
    else:
        rospy.loginfo("\033[1;31mTEST: FAILED\033[1;37m")
    state = state + 1
    is_occupied = False


if __name__ == '__main__':
    pub = rospy.Publisher("/dsr01m1013/grasp_complex", Pose, queue_size=10)
    rospy.init_node('grasp_complex_test', anonymous=True)

    rospy.Subscriber("grasp_complex_result", Bool, callback_answers)
    rate = rospy.Rate(10)

    while (not rospy.is_shutdown() and state <= 10):
        if not is_occupied:
            if state == 0:
                print("test 1")
                # Test pour le leeloo reel a sycodal fonctionnelle
                msg = Pose()
                msg.position.x = 18.221
                msg.position.y = 11.8646
                msg.position.z = 1.0
                msg.orientation.x = 1.0
                msg.orientation.y = 0.0
                msg.orientation.z = 0.0
                msg.orientation.w = 0.0

            elif state == 1:
                # Test de leeloo en simulation
                print("msg send")
                msg = Pose()
                msg.position.x = 16.93
                msg.position.y = 2.45
                msg.position.z = 0.90
                msg.orientation.x = 0 #0.27494521386526327
                msg.orientation.y = 0 #0.26671210289381225
                msg.orientation.z = 0 #0.6430878659086348
                msg.orientation.w = 1.0 #0.6631046525727469



            else:
                break
            pub.publish(msg)
            is_occupied = True
            print("\033[1;37mmsg pub: ", str(state))
        rate.sleep()
