#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from utility.doosan import Doosan

octo_pub = rospy.Publisher('/toggle_octomap', Bool, queue_size=10)

if __name__ == '__main__':
    rospy.init_node('init_octomap', anonymous=True)
    arm = Doosan()
    # rospy.sleep(1)

    octo_pub.publish(True)
    rospy.sleep(5)
    arm.clear_collisions()
    octo_pub.publish(False)


