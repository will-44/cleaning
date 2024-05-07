#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String

"""
Basic class example of a behavior in the HBBA architecture
"""
# a motivation is usually a sequence of desire to achieve a high level goal
# it can be implemented as a finite state machine, each state add desires to activate perception and behavior
# modules in order to achieve the motivation goal
class BasicMotivation():
    def __init__(self):

        self.test_state = rospy.Subscriber("scenario_1", Bool, self.start_scenario_cb) # Event to initiate the scenario
        self.mir_scenario = rospy.Subscriber("scenario_mir", Bool, self.start_senario_mir) # Event to initiate the scenario
        self.pub_arm = rospy.Publisher("desire", String, queue_size=10) # Publish the desires to the intention workspace
        self.pub_mir = rospy.Publisher("desire", String, queue_size=10) # Publish the desires to the intention workspace


    # the callback functions are used to "navigate" through the state of the scenario sequence
    # further work is required in this file...
    def start_scenario_cb(self, _):
        print(__file__ + " started scenario1")
        self.pub_arm.publish("arm_motion")
    def start_senario_mir(self, _):
        print(__file__ + " started mir")
        self.pub_mir.publish("mir_motion")


if __name__ == "__main__":
    try:
        rospy.init_node("scenario_1")
        node = BasicMotivation()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass