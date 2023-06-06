#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from hbba_common.publishers import OnOffHbbaPublisher
from hbba_common.subscribers import OnOffHbbaSubscriber
"""
Basic class example of a behavior in the HBBA architecture
"""
class BasicArmBehavior():
    def __init__(self):

        # Using the OnOffHbba publisher ans subscriber allow the topic to be filtered and its activation managed by desires
        self.pub_joint = OnOffHbbaPublisher("/dsr01m1013/move_j", JointState, queue_size=10)
        OnOffHbbaSubscriber("perception_output", Bool, self.publisher_function_callback)

        self.sign = 1 # just a variable to alternate movement for demonstration purpose

    # the callback functions are used to produce the action sequence of the behavior
    # It can be an inner state machine, a reaction to incoming data
    # or can be used to update the inner states of the behavior
    # In this example we just alternate the robot arm between 2 positions
    def publisher_function_callback(self, _):
        """
        Basic callback function of a behavior reacting to a perception
        """

        joint = JointState()

        joint.position.append(self.sign*1.519094383)
        joint.position.append(self.sign*1.162374874)
        joint.position.append(self.sign*-1.94320531)
        joint.position.append(self.sign*0.06055833159)
        joint.position.append(self.sign*1.53860536)
        joint.position.append(self.sign*-3.062244302)

        self.sign = -self.sign

        self.pub_joint.publish(joint)

        print("I published")



if __name__ == "__main__":

    try:
        rospy.init_node('basic_arm_behavior', anonymous=False)
        armBehavior = BasicArmBehavior()
        
        rospy.spin()
        

    except rospy.ROSInterruptException:
        print("except")