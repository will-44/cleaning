#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from std_msgs.msg import Empty

'''
This file propos a solution to mesure the accuracy of leeloo (not finish)
'''
state = 1
is_occupied = False
is_issue = False

def callback_answers(answers):
    global state, is_occupied
    expected = True

    if answers.data == expected:
        rospy.loginfo("\033[1;32mTEST: PASSED\033[1;37m")
    else:
        is_issue = True
        rospy.loginfo("\033[1;31mTEST: FAILED\033[1;37m")

    is_occupied = False


if __name__ == '__main__':
    grasp_pub = rospy.Publisher("grasp", Pose, queue_size=10)
    mir_pub = rospy.Publisher("mir_go_to", Pose2D, queue_size=10)
    home_pub = rospy.Publisher("home", Empty, queue_size=10)
    rospy.init_node('precision_test', anonymous=True)

    rospy.Subscriber("grasp_result", Bool, callback_answers)
    rospy.Subscriber("mir_result", Bool, callback_answers)
    rospy.Subscriber("home_result", Bool, callback_answers)
    rate = rospy.Rate(1)



    # For security, in reel test, we only do the first test
    while (not rospy.is_shutdown() and not is_issue):

        if not is_occupied:
            if state == 0:
                # Go to position 1 for the mir
                msg = Pose2D()
                msg.x = 25.023 # 13.5#
                msg.y = 12.881 # 7.2#
                msg.theta = 1.53588974 #1.57 #
                print(msg)
                state = 1
                mir_pub.publish(msg)

            elif state == 1:
                #  Prepare arm
                msg = Pose()
                msg.position.x = -9.605680465698242 / 1000
                msg.position.y = 358.61541748046875 / 1000
                msg.position.z = 733.8211669921875 / 1000
                msg.orientation.x = 88.45662689208984 * math.pi / 180
                msg.orientation.y = -46.79851150512695 * math.pi / 180
                msg.orientation.z = -178.5690460205078 * math.pi / 180
                state = 2
                print(msg)
                grasp_pub.publish(msg)

            elif state == 2:
                #  Put the stylo on the wall
                msg = Pose()
                msg.position.x = -0.03607650375366211
                msg.position.y = -0.0192667236328125
                msg.position.z = 1.089222412109375
                msg.orientation.x = 1.500983156711400035
                msg.orientation.y = -0.8203047484352999907
                msg.orientation.z = -3.0892327760223001754
                state = 1
                print(msg)
                grasp_pub.publish(msg)


            elif state == 3:

                #  Premove arm
                msg = Pose()
                msg.position.x = -9.605680465698242 / 1000
                msg.position.y = 358.61541748046875 / 1000
                msg.position.z = 733.8211669921875 / 1000
                msg.orientation.x = 88.45662689208984 * math.pi / 180
                msg.orientation.y = -46.79851150512695 * math.pi / 180
                msg.orientation.z = -178.5690460205078 * math.pi / 180
                state = 4
                print(msg)
                grasp_pub.publish(msg)

            elif state == 4:
                # Test de pose valide
                msg = Empty()
                state = 5
                print(msg)
                home_pub.publish(msg)

            elif state == 5:

                # Mir go to an other position
                msg = Pose2D()
                msg.x = 25.023 #7.2#
                msg.y = 11.278 #3.5#
                msg.theta = -1.46607657 # 1.57#
                state = 0
                print(msg)
                mir_pub.publish(msg)

            else:
                break

            is_occupied = True

        rate.sleep()
