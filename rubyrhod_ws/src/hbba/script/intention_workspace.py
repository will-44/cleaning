#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from hbba.srv import SetOnOffFilterState
from hbba_common.filter_states import OnOffHbbaFilterState, ThrottlingHbbaFilterState
from hbba_common.subscribers import OnOffHbbaSubscriber

class IntentionWorkspace():
    def __init__(self):
        rospy.Subscriber("desire", String, self.add_desire)

        self.current_desires = []

        # this table associates all desires to topics so when the desire is added/removed the filters are turned
        # TODO implement a better way to manage the topics to each modules
        self.desire_table = {
        "arm_motion": ["/dsr01m1013/move_j", "perception_output"],
        "mir_motion": ["/mir_go_to", "perception_mir"],
        "any_desire": ["any_topic_name", "any_other_topic_name", "and_so_on"],
        }
    
    # When desired the filter associated to a behavior or perception are turned off
    # the OnOffHBBA subscriber/publisher create a service to trigger the filtering
    # This function call this service for the appropriate topics
    def add_desire(self, desire_string):

        desire = desire_string.data
        self.current_desires.append(desire)

        print(__file__ + " added desire " + desire)

        topic_list_set_filter_off = self.desire_table[desire]

        for topic in topic_list_set_filter_off:
            service_name = topic + "/filter_state"
            print(service_name)
            rospy.wait_for_service(service_name)
            service_filter = rospy.ServiceProxy(service_name, SetOnOffFilterState)
            try:
                service_filter(False)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))




if __name__ == "__main__":
    try:
        rospy.init_node("intention_workspace")
        node = IntentionWorkspace()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass