#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from hbba_common.publishers import OnOffHbbaPublisher
from hbba_common.subscribers import OnOffHbbaSubscriber
import yaml
import rospkg
from std_msgs.msg import Float64
from std_msgs.msg import String

"""
Basic class example of a behavior in the HBBA architecture
"""

class BasicMirMove():
    def __init__(self):

        path = rospkg.RosPack().get_path('behaviors') + "/config/mir_waypoint.yaml"
        print(path)
        with open(path, 'r') as stream:
            out = yaml.load(stream, Loader=yaml.FullLoader)
            self.poses = list(out.values())

        self.pub_mir = OnOffHbbaPublisher("/mir_go_to", Pose2D, queue_size=10)
        OnOffHbbaSubscriber("perception_mir", Bool, self.publisher_function_callback)
        rospy.Subscriber("/proc_battery_level", Float64, self.callback_bettery_relay)
        rospy.Subscriber("/proc_battery_relay", String, self.callback_bettery_lv)
        self.is_charging = "Off"
        self.battery_lv = 0
        self.iter_pose = 0

        # launch the fist move, TMP
        pose = Pose2D()
        pose.x = self.poses[self.iter_pose][0]
        pose.y = self.poses[self.iter_pose][1]
        pose.theta = self.poses[self.iter_pose][2]

        self.pub_mir.publish(pose)

        print("I published")
        self.iter_pose += 1
        self.iter_pose %= len(self.poses)


    def callback_bettery_relay(self, msg):
        self.is_charging = msg

    def callback_bettery_lv(self, msg):
        self.battery_lv = 0

    def publisher_function_callback(self, _):
        """
        Basic callback function of a behavior reacting to a perception
        """
        if self.battery_lv >= 35 or self.is_charging != "On":
            pose = Pose2D()
            pose.x = self.poses[self.iter_pose][0]
            pose.y = self.poses[self.iter_pose][1]
            pose.theta = self.poses[self.iter_pose][2]

            self.pub_mir.publish(pose)

            print("I published")
            self.iter_pose += 1
            self.iter_pose %= len(self.poses)

if __name__ == "__main__":

    try:
        rospy.init_node('mir_move_behavior', anonymous=False)
        armBehavior = BasicMirMove()

        rospy.spin()


    except rospy.ROSInterruptException:
        print("except")
