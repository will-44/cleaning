#!/usr/bin/env python3

# Standard python submodules
import subprocess
import sys
import os
import traceback
from typing import final
from unittest import result
import uuid
import argparse
import json

# ROS imports
import rospy
from nav_msgs.msg import Odometry

# Azure IoT device import
from azure.iot.device import IoTHubDeviceClient, Message

class AzureUptimeTelemetry:
    """This node sends weather or not the robot is moving every [period] secconds
    """

    def __init__(self, period):
        """Node used to send telemetry weather or not the robot is moving every [period] secconds

        :param period: time interval between sending two telemetry messages
        :type period: int
        """
        
        # Device client to connect to IoTHub
        rospy.loginfo('{nodeName} : Creating IoTHub device client...'.format(nodeName = rospy.get_name()))
        self._device_client = IoTHubDeviceClient.create_from_connection_string(os.getenv('IOTHUB_DEVICE_CONNECTION_STRING'))
        rospy.loginfo('{nodeName} : Client created'.format(nodeName = rospy.get_name()))

        # Shutdown procedure to be executed when killing node
        rospy.loginfo('{nodeName} : Linking shutdown method...'.format(nodeName = rospy.get_name()))
        rospy.on_shutdown(self.__shutdown)
        rospy.loginfo('{nodeName} : Shutdown method linked'.format(nodeName = rospy.get_name()))

        # Throttling node from topic_tools used to slow the rate of the /odom topic to a specified period
        rospy.loginfo('{nodeName} : Starting throttling node for /odom topic at a rate of {period}s...'.format(period = period, nodeName = rospy.get_name()))
        subprocess.Popen('rosrun topic_tools throttle messages /odom {frequency} telemetry/odom_throttle __name:=odom_throttle __ns:={nameSpace}'.format(frequency=1/period, nameSpace=rospy.get_name()), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        rospy.loginfo('{nodeName} : Throttling node started'.format(nodeName = rospy.get_name()))

        # Connecting to IoT hub
        rospy.loginfo('{nodeName} : Connecting to IoT Hub...'.format(nodeName = rospy.get_name()))
        self._device_client.connect()
        rospy.loginfo('{nodeName} : Connected to IoT Hub'.format(nodeName = rospy.get_name()))

        # Susbscribing to throttled /odom topic
        rospy.loginfo('{nodeName} : Subscribing to throttled /odom topic...'.format(nodeName = rospy.get_name()))
        rospy.Subscriber('{nodeName}/telemetry/odom_throttle'.format(nodeName = rospy.get_name()), Odometry, self.send_is_up)
        rospy.loginfo('{nodeName} : Subscribed to throttled /odom topic'.format(nodeName = rospy.get_name()))

    # def __del__(self):
    def __shutdown(self):
        """Releasing ressources used by this node
            Killing thrilling node
            Shuting down IoT device client
        """

        # Disconnecting from IoT hub
        rospy.loginfo('{nodeName} : Disconnecting from IoT hub...'.format(nodeName = rospy.get_name()))
        self._device_client.disconnect()
        rospy.loginfo('{nodeName} : Disconnected from IoT hub'.format(nodeName = rospy.get_name()))

        # The IoT DeviceClient needs to be shutted down before destruction for graceful exit
        rospy.loginfo('{nodeName} : Shuting down IoT device client...'.format(nodeName = rospy.get_name()))
        self._device_client.shutdown()
        rospy.loginfo('{nodeName} : Shutted down IoT device client'.format(nodeName =rospy.get_name()))

        # Stop throttling via a rosnode kill of the created node
        rospy.loginfo('{nodeName} : Killing throttling node for /odom topic...'.format(nodeName = rospy.get_name()))
        subprocess.run('rosnode kill {nameSpace}/odom_throttle'.format(nameSpace=rospy.get_name()), shell=True)
        rospy.loginfo('{nodeName} : Killed throttling node for /odom topic'.format(nodeName = rospy.get_name()))

    def send_is_up(self, data=Odometry):
        """Sends D2C message containing if the robot is currently moving

        :param data: odometry message, contains current mouvement of the robot
        :type data: nav_msgs/Odometry
        """

        # If any components of the twist are non-zero, then the robot is considered to be moving
        if (
            data.twist.twist.linear.x or
            data.twist.twist.linear.y or
            data.twist.twist.linear.z or
            data.twist.twist.angular.x or
            data.twist.twist.angular.y or
            data.twist.twist.angular.z
        ):
            msg = Message(json.dumps({"isMoving":1}))
        # Else the robot is considered to be immobile
        else:
            msg = Message(json.dumps({"isMoving":0}))

        msg.message_id = uuid.uuid4()
        msg.correlation_id = 'leeloo-' + str(msg.message_id)
        msg.content_encoding = 'utf-8'
        msg.content_type = "application/json"

        rospy.loginfo('{nodeName} : Sending message'.format(nodeName = rospy.get_name()) + str(msg) + '...')
        self._device_client.send_message(msg)
        rospy.loginfo('{nodeName} : Message sent'.format(nodeName = rospy.get_name()))

if __name__ == "__main__":
    parser  = argparse.ArgumentParser(description="send mouvement telemetry data every [--period, -p] seconds")
    parser.add_argument("-p", "--period", help="period in second between image recording", type=int, default=10)
    # Apply arguments specific to ROS, returns the remaining arguments
    argv = rospy.myargv(sys.argv)
    # Parse arguments, first argument is ignore as it is the name of the script
    args=parser.parse_args(argv[1:])

    try:
        rospy.loginfo('{nodeName} : Connecting to ROSMaster...'.format(nodeName = rospy.get_name()))
        rospy.init_node('azure_uptime_telemetry', anonymous=False)
        rospy.loginfo('{nodeName} : Connected to ROSMaster'.format(nodeName = rospy.get_name()))

        node = AzureUptimeTelemetry(args.period)

        rospy.loginfo('{nodeName} : Uptime telemetry running'.format(nodeName = rospy.get_name()))
        rospy.spin()
        rospy.loginfo('{nodeName} : Shuting down uptime telemetry'.format(nodeName = rospy.get_name()))
    
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())
