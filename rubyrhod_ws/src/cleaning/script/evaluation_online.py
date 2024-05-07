#!/usr/bin/env python3

import json
import math
import rospy
from utility.doosan import Doosan
import rospkg
from rospy_message_converter import message_converter

rospy.init_node('evaluation', anonymous=True)

arm = Doosan()
studentsList = []
# print("Started Reading JSON file which contains multiple JSON document")
file_path = rospkg.RosPack().get_path('cleaning') + f"/data/telemetrie/telemetrie_15_18.json"
with open(file_path) as f:
    for jsonObj in f:
        studentDict = json.loads(jsonObj)
        studentsList.append(studentDict)

print("Printing each JSON Decoded Object")

# position = [-1.3362817909675153, -0.5145970941712082, -1.4175696845056331, 5.355658318761093, -1.072574209592224, -1.5146858906611216]
# position = [-1.4443501409853998, -0.4141585314415087, -1.4173241654286248, 5.607104608567272, -1.445039121320586, -1.310685835599539]
# position = [0.5549236137567284, -0.2253599489704838, 1.865123754850512, 3.14116481794196, -1.3472628216448745, -3.1386177985256825]
# P_index = arm.get_joint_limit_index(position)
# print(P_index)
# mani = arm.get_manipulability(position)
# print(mani)
# arm.go_to_j(position)

for student in studentsList:
    if "position" in student.keys():
        student = json.dumps(student)
        # print(student)
        # joint_state_data = ros.deserialize_message('sensor_msgs/JointState', student)
        joint_state_data = message_converter.convert_dictionary_to_ros_message('sensor_msgs/JointState', json.loads(student))
        # print(joint_state_data)
        P_index = arm.get_joint_limit_index(joint_state_data.position)
        print(P_index)
        mani = arm.get_manipulability(joint_state_data.position)
        print(mani)
        print("---------------------------------")
        arm.go_to_j(joint_state_data.position)
        while arm.check_motion() != 0:
            rospy.sleep(2)
