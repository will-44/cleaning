#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool


class MirPerception():
    def __init__(self):
        rospy.Subscriber("/mir_result", Bool, self.callback)
        rospy.Subscriber("/start_mir_mission", Bool, self.callback)
        self.pub_mir = rospy.Publisher('/perception_mir', Bool, queue_size=10)

    def callback(self, data):
        self.pub_mir.publish(True)


if __name__ == "__main__":

    try:
        rospy.init_node('mir_perception', anonymous=False)
        mirPerception = MirPerception()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("except")
