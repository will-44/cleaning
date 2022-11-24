#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

'''
This test send a position to the node grasp_complexe to grasp an object far from the robot
and listen the result of the operation
'''
state = 0
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
    pub = rospy.Publisher("grasp_complex", Pose, queue_size=10)
    rospy.init_node('grasp_complex_test', anonymous=True)

    rospy.Subscriber("grasp_complex_result", Bool, callback_answers)
    rate = rospy.Rate(10)
    # For security, in reel test, we only do the first test
    while (not rospy.is_shutdown() and state <= 0):
        if not is_occupied:
            if state == 0:
                # Test fonctionnelle
                msg = Pose()
                msg.position.x = 25
                msg.position.y = 12
                msg.position.z = 1.0
                msg.orientation.x = 0
                msg.orientation.y = 0
                msg.orientation.z = 1.57

            elif state == 1:
                # Test de pose non valide
                msg = Pose()
                msg.position.x = 23.95
                msg.position.y = 9.15
                msg.position.z = 0.64917
                msg.orientation.x = 0.27494521386526327
                msg.orientation.y = 0.26671210289381225
                msg.orientation.z = 0.6430878659086348
                msg.orientation.w = 0.6631046525727469

            elif state == 2:
                # Test de pose valide avec objet sur table 
                msg = Pose()
                msg.position.x = 23.95
                msg.position.y = 11
                msg.position.z = 1.0
                msg.orientation.x = 0.0
                msg.orientation.y = 0.0
                msg.orientation.z = 0.0
                msg.orientation.w = 1.0

            elif state == 3:
                rospy.loginfo("on envoie e bon msg")
                # Test fonctionnelle en simulation
                msg = Pose()
                msg.position.x = 14.5
                msg.position.y = 9.12
                msg.position.z = 1.5
                msg.orientation.x = 0
                msg.orientation.y = 0
                msg.orientation.z = 0.707
                msg.orientation.w = 0.707

            else:
                break
            pub.publish(msg)
            is_occupied = True
            print("\033[1;37mmsg pub: ", str(state))
        rate.sleep()
