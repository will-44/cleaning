#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray

"""
Perception module used for battery level monitoring, the modules signals when the battery level is low
"""
class BatteryLevel():
    def __init__(self):

        # self.pub_joint = OnOffHbbaPublisher("/dsr01m1013/move_j", JointState, queue_size=10)
        # OnOffHbbaSubscriber("perception_output", Bool, self.publisher_function_callback)
        rospy.Subscriber("diagnostics", DiagnosticArray, self.callback)
        self.pub_level = rospy.Publisher("/proc_battery_level", Float64, queue_size=10)
        self.pub_relay = rospy.Publisher("/proc_battery_relay", String, queue_size=10)
        self.battery_level = 100.0
        self.remaining_time_sec = 0
        self.battery_voltage = 0
        self.remaining_time_hour_minute_sec = 0
        self.battery_amps = 0
        self.battery_relay = "Off"
        print("created battery level")


    def callback(self, msg):
        """
        callback function that process the system diagnostic informations 
        to retreive the battery level in the diagnostic array
        :param msg: diagnostic array received, described in
        http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html
        """

        try:
            if msg.status[2].name == "battery_node: Battery":

                self.battery_voltage = float(msg.status[2].values[0].value)
                self.battery_level = float(msg.status[2].values[1].value)
                self.remaining_time_sec = float(msg.status[2].value[2].value)
                self.remaining_time_hour_minute_sec = str(msg.status[2].value[3].value)# [HH:MM:SS]
                self.battery_amps = float(msg.status[2].values[4].value)
        except:
            pass

        try:
            if msg.status[0].name == "battery_node: Charging Status":

                self.battery_relay = msg.status[0].values[0].value
                print(msg.status[0].values[0].value)
        except:
            pass

        self.pub_level.publish(self.battery_level)
        self.pub_relay.publish(self.battery_relay)



if __name__ == "__main__":

    try:
        rospy.init_node('proc_battery_level', anonymous=False)
        batteryLevel = BatteryLevel()
        
        rospy.spin()
        

    except rospy.ROSInterruptException:
        print("except")

