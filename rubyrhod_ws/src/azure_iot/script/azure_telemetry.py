#!/usr/bin/env python3

# Standard python submodules
import argparse
import json
import os
import psutil
import queue
import subprocess
import sys
import threading
import traceback
from typing import Dict, final
from unittest import result
import uuid

# ROS imports
import rospy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
import rosgraph

# Azure IoT device import
from azure.iot.device import IoTHubDeviceClient, Message

class AzureUptimeTelemetry:
    """This node sends telemetry data to IoT Hub every [period] secconds
    
    Telemetry signals are :
        whether or not a rosmaster runs (heartbeat)
        whether or not the robot is moving
        current ressource usage (cpu, usage)
        current battery utilisation (is charging, voltage, remaining charge, remaining time)
    """

    # Construtor of class, initiates IoT Hub connection and variables and connect callback functions
    # to timers and topic subscriptions
    def __init__(self, period):
        """This node sends telemetry data to IoT Hub every [period] secconds
    
        Telemetry signals are :
            whether or not a rosmaster runs (heartbeat)
            whether or not the robot is moving

        :param period: time interval between sending two telemetry messages
        :type period: int
        """

        # Saving period
        self._period = period

        # Shutdown procedure to be executed when killing node
        rospy.loginfo('{nodeName} : Linking shutdown method...'.format(nodeName = rospy.get_name()))
        rospy.on_shutdown(self.__shutdown)
        rospy.loginfo('{nodeName} : Shutdown method linked'.format(nodeName = rospy.get_name()))
        
        # Device client to connect to IoTHub
        rospy.loginfo('{nodeName} : Creating IoTHub device client...'.format(nodeName = rospy.get_name()))
        self._device_client = IoTHubDeviceClient.create_from_connection_string(os.getenv('IOTHUB_DEVICE_CONNECTION_STRING'))
        rospy.loginfo('{nodeName} : Client created'.format(nodeName = rospy.get_name()))

        # Connecting to IoT hub
        rospy.loginfo('{nodeName} : Connecting to IoT Hub...'.format(nodeName = rospy.get_name()))
        self._device_client.connect()
        rospy.loginfo('{nodeName} : Connected to IoT Hub'.format(nodeName = rospy.get_name()))

        # Creating queue for messages to be sent
        self._msg_queue = queue.Queue()

        # Creating and started worker thread
        self._thread = threading.Thread(target = self.send_msg_worker, name = 'send_mesg_worker_thread', daemon = True)
        self._thread.start()

        # Initiating battery telemetetry save space to insure it exists
        self._battery_telemetry_msg = None

        # Throttling node from topic_tools used to slow the rate of the /odom topic to a specified period
        rospy.loginfo('{nodeName} : Starting throttling node for /odom topic at a rate of {period}s...'.format(period = period, nodeName = rospy.get_name()))
        subprocess.Popen('rosrun topic_tools throttle messages /odom {frequency} telemetry/odom_throttle __name:=odom_throttle __ns:={nameSpace}'.format(frequency=1/period, nameSpace=rospy.get_name()), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        rospy.loginfo('{nodeName} : Throttling node started'.format(nodeName = rospy.get_name()))

        # Susbscribing to throttled /odom topic
        rospy.loginfo('{nodeName} : Subscribing to throttled /odom topic...'.format(nodeName = rospy.get_name()))
        rospy.Subscriber('{nodeName}/telemetry/odom_throttle'.format(nodeName = rospy.get_name()), Odometry, self.send_is_moving)
        rospy.loginfo('{nodeName} : Subscribed to throttled /odom topic'.format(nodeName = rospy.get_name()))

        # Subscribing to /diagnostics
        rospy.loginfo('{nodeName} : Subscribing to /diagnostics topic...'.format(nodeName = rospy.get_name()))
        rospy.Subscriber('/diagnostics', DiagnosticArray, self.get_battery_telemetry)
        rospy.loginfo('{nodeName} : Subscribed to /diagnostics topic'.format(nodeName = rospy.get_name()))

        # Calling heartbeat every [period] seconds
        rospy.loginfo('{nodeName} : Sending heartbeat every {period}s'.format(nodeName = rospy.get_name(), period = period))
        self._heartbeat_timer = rospy.Timer(rospy.Duration(period), self.send_heartbeat)

        # Sending system usage every [period] seconds
        rospy.loginfo('{nodeName} : Sending system usage every {period}s'.format(nodeName = rospy.get_name(), period = period))
        self._system_usage_timer = rospy.Timer(rospy.Duration(period), self.send_system_usage)

        # Sending battery telemtry every [period] seconds
        rospy.loginfo('{nodeName} : Sending battery telemetry every {period}s'.format(nodeName = rospy.get_name(), period = period))
        self._battery_telemetry_timer = rospy.Timer(rospy.Duration(period), self.send_bettery_telemetry)

    # Acts as a destructor, liked to the ropsy.on_shutdown method, will be invoked when the node is getting shut down
    # def __del__(self):
    def __shutdown(self):
        """Releasing ressources used by this node
            Killing throttling node
            Shuting down IoT device client
        """

        # Stop throttling via a rosnode kill of the created node
        rospy.loginfo('{nodeName} : Killing throttling node for /odom topic...'.format(nodeName = rospy.get_name()))
        subprocess.run('rosnode kill {nameSpace}/odom_throttle'.format(nameSpace=rospy.get_name()), shell=True)
        rospy.loginfo('{nodeName} : Killed throttling node for /odom topic'.format(nodeName = rospy.get_name()))

        # Stopping heartbeat
        rospy.loginfo('{nodeName} : Stopping sending of heartbeat'.format(nodeName = rospy.get_name()))
        self._heartbeat_timer.shutdown()
        rospy.loginfo('{nodeName} : Sending of heartbeat stopped'.format(nodeName = rospy.get_name()))

        # Stopping system usage
        rospy.loginfo('{nodeName} : Stopping sending of system usage'.format(nodeName = rospy.get_name()))
        self._system_usage_timer.shutdown()
        rospy.loginfo('{nodeName} : Sending of system usage stopped'.format(nodeName = rospy.get_name()))

        # Stopping battery telemetry
        rospy.loginfo('{nodeName} : Stopping sending of battery telemetry'.format(nodeName = rospy.get_name()))
        self._battery_telemetry_timer.shutdown()
        rospy.loginfo('{nodeName} : Sending of battery telemetry stopped'.format(nodeName = rospy.get_name()))

        # Waitng for message queue to be empty
        rospy.loginfo('{nodeName} : Waiting for message queue to be empty...'.format(nodeName = rospy.get_name()))
        while(not self._msg_queue.empty()):
            rospy.loginfo('{nodeName} : Message queue still has {nb_msgs} left to process'.format(nodeName = rospy.get_name(), nb_msgs = self._msg_queue.qsize()))
        self._msg_queue.join()
        rospy.loginfo('{nodeName} : Message queue is empty'.format(nodeName = rospy.get_name()))

        # Disconnecting from IoT hub
        rospy.loginfo('{nodeName} : Disconnecting from IoT hub...'.format(nodeName = rospy.get_name()))
        self._device_client.disconnect()
        rospy.loginfo('{nodeName} : Disconnected from IoT hub'.format(nodeName = rospy.get_name()))

        # The IoT DeviceClient needs to be shutted down before destruction for graceful exit
        rospy.loginfo('{nodeName} : Shuting down IoT device client...'.format(nodeName = rospy.get_name()))
        self._device_client.shutdown()
        rospy.loginfo('{nodeName} : Shutted down IoT device client'.format(nodeName =rospy.get_name()))

    #---Callback functions---#
    # Callback function, invoked every time a message is published on topic
    # {namespace}/telemetry/odom_throttle, this topic is throttled to 1 every [period] seconds
    def send_is_moving(self, data : Odometry):
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
            payload = {"isMoving":1}
        # Else the robot is considered to be immobile
        else:
            payload = {"isMoving":0}

        self.send_msg_to_iothub(payload)

    # Callback function, invoked every time a message is published on topic /Diagnostics
    def get_battery_telemetry(self, data : DiagnosticArray):
        """Save the most recent diagnostic message concerning battery
            Saved message will be used later to send telemetry to IoTHub

        :param data: Diagnostic message, might contain data related to the battery
        :type data: DiagnosticArray
        """

        if(data.status[0].hardware_id == "Battery"):
            self._battery_telemetry_msg = data
    
    # Callback function, invoked by timer _battery_telemetry_timer every [period] seconds
    def send_bettery_telemetry(self, _):
        """Send battery telemetry containing :
            isCharging
            batteryVoltage
            remainingBatteryCapacity
            remainingBatteryTime
        """
        if(self._battery_telemetry_msg is not None):
            battery_telemetry_msg = self._battery_telemetry_msg

            payload = {
                'isCharging' : 1 if battery_telemetry_msg.status[0].values[0].value == 'On' else 0,
                'batteryVoltage' : float(battery_telemetry_msg.status[2].values[0].value),
                'remainingBatteryCapacity' : float(battery_telemetry_msg.status[2].values[1].value),
                'remainingBatteryTime' : int(battery_telemetry_msg.status[2].values[2].value)
            }

            self.send_msg_to_iothub(payload)

    # Callback function, invoked by timer _heartbeat_timer every [period] seconds
    def send_heartbeat(self, _):
        """Sends D2C message containing if a ROSMaster is online and, therefore, if the robot is alive
        """
        if(rosgraph.is_master_online()):
            payload = {'isUp':1}
        else:
            payload = {'isUp':0}
        
        self.send_msg_to_iothub(payload)

    # Callback function, invoked by timer _system_usage_timer every [period] seconds
    def send_system_usage(self, _):
        """Sends D2C message containing CPU and memory usage
        """
        payload = {"cpuUsage": psutil.cpu_percent(interval=1),
                    "memoryUsage": psutil.virtual_memory().percent}

        self.send_msg_to_iothub(payload)

    #---Message sending mechanisms---#
    # Worker function running in daemon thread send_mesg_worker_thread
    def send_msg_worker(self):
        while True:
            # Retrieving first item in queue
            msg = self._msg_queue.get()

            # Sending message to IoT Hub
            rospy.loginfo('{nodeName} : Sending message {message} with correlation ID {correlation_id}...'.format(nodeName = rospy.get_name(), message = str(msg), correlation_id = str(msg.correlation_id)))
            self._device_client.send_message(msg)
            rospy.loginfo('{nodeName} : Sent message with correlation ID {correlation_id}'.format(nodeName = rospy.get_name(), correlation_id = str(msg.correlation_id)))

            # Inform queue that task is done
            self._msg_queue.task_done()

    # Invoked by functions sending a message to add tht message to the queue
    def send_msg_to_iothub(self, payload: Dict):
        """Generic function to send a payload to IoT Hub as a device to cloud message

        Messages are added to a queue to be send by worker thread

        :param payload: payload that will be sent to the IoT Hub
        :type payload: json-like Dict
        """

        payload["period"]=self._period

        msg = Message(json.dumps(payload))

        msg.message_id = uuid.uuid4()
        msg.correlation_id = 'leeloo-' + str(msg.message_id)
        msg.content_encoding = 'utf-8'
        msg.content_type = "application/json"

        # Adding message to queue so it can be sent by worker thread
        self._msg_queue.put(msg)

# Script entry point if invoked as executable
if __name__ == "__main__":
    parser  = argparse.ArgumentParser(description="send telemetry data every [--period, -p] seconds")
    parser.add_argument("-p", "--period", help="period in second between telemetry messages", type=int, default=10)
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
