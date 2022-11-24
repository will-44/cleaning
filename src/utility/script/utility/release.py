#!/usr/bin/env python3

import rospy
from doosan import Doosan
from robotiq import Robotiq
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

def callback(data):
    '''

    :param data: The position of the object geometry_msgs.msg.Pose
    :return: Send a message on the topics grasp_result. True if the action is done or False if some issue append
    '''
    result = False
    arm = Doosan()
    gripper = Robotiq()
    pub = rospy.Publisher('release_result', Bool, queue_size=10)


    # The quaternion is not considered for now
    pos = [data.position.x, data.position.y, data.position.z,
           data.orientation.x, data.orientation.y, data.orientation.z]
    result = arm.go_to_l(pos)
    while((arm.check_motion() != 0)):
        rospy.sleep(1)

    gripper.open()
    while(gripper.status().gPR != 0):
        rospy.sleep(1)

    pub.publish(result)

if __name__ == '__main__':
    rospy.init_node('release_node', anonymous=True)
    rospy.Subscriber("release", Pose, callback)

    # just activate the gripper
    gripper = Robotiq()
    gripper.reset()
    rospy.sleep(2)
    gripper.activate()
    rospy.sleep(5)
    rospy.spin()
