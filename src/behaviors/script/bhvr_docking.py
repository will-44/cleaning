#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import requests
import json


BATTERY_LEVEL_LOW_THRESHOLD = 30.0

# Get Request
ip = "192.168.100.209" # Ip of the fleet
host = "http://" + ip + "/api/v2.0.0/"

# Format headers
headers = {}
headers["Content-Type"] = "application/json"
headers["Authorization"] = "Basic TWlyX3N5Y29kYWw6ODViYTQzZDhhM2QzZjE3MDg5YjkzOWEyY2NlZmUxNTBmNTk5NDg2YzI1MzEwODRkN2VkZGQwNDFmOGRiODE0Ng=="

"""
behavior module used to send the robot to its charge station when the battery level is low
"""
class DockingBehavior():
    def __init__(self):

        rospy.Subscriber("proc_battery_level", Float64, self.callback)

        get_missions = requests.get(host + "actions", headers=headers)
        # get_missions = requests.delete(host + "mission_queue", headers=headers)
        # print(get_missions.text)
        # print(get_missions.status_code)
        self.has_charged_since_last_docking = True
        self.last_known_battery_charge = 100.0
        self.is_charging = False

        
        print("created docking behavior")


    def callback(self, msg):
        """
        The callback is triggered when the battery level is low and sends the robot to its docking station using the mission api
        """
        battery_charge = msg.data
        if battery_charge > self.last_known_battery_charge:
            self.has_charged_since_last_docking = True

        if battery_charge >= self.last_known_battery_charge:
            self.is_charging = True
        else:
            self.is_charging = False

        if self.has_charged_since_last_docking  == True and battery_charge < BATTERY_LEVEL_LOW_THRESHOLD and self.is_charging == False:
            # The marker in the robot map should be named "Dock"
            # change mission Charger_mir
            mission_id = {"mission_id": "441c9308-fbd8-11ec-a605-94c691a7398b"} # Charger_mir
            post_mission  = requests.post(host + "mission_queue", json=mission_id, headers=headers)
            print(post_mission.text)
            print(post_mission.status_code)
            print("sent docking mission")

            self.has_charged_since_last_docking = False

        self.last_known_battery_charge = battery_charge



if __name__ == "__main__":

    try:
        rospy.init_node('bhvr_docking', anonymous=False)
        dockingBehavior = DockingBehavior()
        
        rospy.spin()
        

    except rospy.ROSInterruptException:
        print("except")

