#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

"""
Basic class example of a behavior in the HBBA architecture
"""
class TestBehavior2():
    def __init__(self):

        r = rospy.Rate(1)

        self.pub = rospy.Publisher("test", Bool, queue_size=10)

        while not rospy.is_shutdown():
            self.pub.publish(False)
            r.sleep()


if __name__ == "__main__":

    try:
        rospy.init_node('bhvr_test2', anonymous=False)
        testBehavior = TestBehavior2()
        
        rospy.spin()
        

    except rospy.ROSInterruptException:
        print("except")