#!/usr/bin/env python3

import rospy
import rospkg
import csv
from geometry_msgs.msg import Pose, Point, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from ros_numpy import numpify, msgify
import numpy as np
import json
import time
from rospy_message_converter import json_message_converter


class TeleMetrie:
    def __init__(self):
        # Sub
        self.spot_sub = rospy.Subscriber("/telemetrie/spot", PoseStamped, self.callback_spot)
        self.guard_sub = rospy.Subscriber("/telemetrie/guard", JointState, self.callback_guard)
        self.finish_sub = rospy.Subscriber("/telemetrie/finish", Bool, self.callback_finish)
        self.dust_sub = rospy.Subscriber("/telemetrie/dust_pose", PoseStamped, self.callback_dust)

        # Global var
        now = time.strftime("%Y%m%d-%H%M%S")

        self.filename = rospkg.RosPack().get_path('cleaning') + f"/data/telemetrie/telemetrie_{now}.json"
        self.pose_actual_spot = 0
        self.pose_actual_guard = 0
        self.pose_previous_guard = 0
        self.actual_pose = 0
        self.is_cleaning_finish = False
        self.file_name = ''
        rospy.loginfo("Telemetrie is ready")

    def callback_spot(self, msg):
        """
        Get the mobile base destination
        :param msg:
        :return:
        """
        with open(self.filename, 'a') as outfile:
            message_str = json_message_converter.convert_ros_message_to_json(msg) + "\n"
            outfile.write(message_str)

    def callback_guard(self, msg):
        """
        Get the TCP destination for guards
        :param msg:
        :return:
        """
        with open(self.filename, 'a') as outfile:
            message_str = json_message_converter.convert_ros_message_to_json(msg) + "\n"
            outfile.write(message_str)

    def callback_dust(self, msg):
        """
        Get TCP destination to clean dust
        :param msg:
        :return:
        """
        with open(self.filename, 'a') as outfile:
            message_str = json_message_converter.convert_ros_message_to_json(msg) + "\n"
            outfile.write(message_str)

    def callback_finish(self, msg):
        """
        Stop telemetrie recording
        :param msg:
        :return:
        """
        self.is_cleaning_finish = msg.data




if __name__ == '__main__':
    rospy.init_node('telemetrie', anonymous=True)
    tele = TeleMetrie()
    while not tele.is_cleaning_finish:
        rospy.sleep(0.1)
    rospy.loginfo("Telemetrie is done")
