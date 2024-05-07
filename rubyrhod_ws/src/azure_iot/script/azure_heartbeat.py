#!/usr/bin/env python3

# Standard python submodules
import argparse
import json
import os
import sys
import traceback
import uuid

# ROS imports
import rospy
import rosgraph

# Azure IoT device import
from azure.iot.device import IoTHubDeviceClient, Message

class AzureHeartbeat:
    """This node sends weather or not a rosmaster runs every [period] seconds
    """

    def __init__(self, period) -> None:
        """This node sends weather or not a rosmaster runs every [period] seconds

        :param period: time interval between sending two heartbeats
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

        # Connecting device client to IoTHub
        rospy.loginfo('{nodeName} : Connecting to IoT Hub...'.format(nodeName = rospy.get_name()))
        self._device_client.connect()
        rospy.loginfo('{nodeName} : Connected to IoT Hub'.format(nodeName = rospy.get_name()))

        # Calling heartbeat every [period]
        rospy.loginfo('{nodeName} : Sending heartbeat every {period}s'.format(nodeName = rospy.get_name(), period = period))
        self._timer = rospy.Timer(rospy.Duration(period), self.send_heartbeat)

    def __shutdown(self):
        """Releasing ressources used by this node
            Killing thrilling node
            Shuting down IoT device client
        """

        # Stopping heartbeat, but waiting for completion if a heartbeat is getting sent
        rospy.loginfo('{nodeName} : Stopping sending of heart heartbeat'.format(nodeName = rospy.get_name()))
        self._timer.shutdown()
        self._timer.join()
        rospy.loginfo('{nodeName} : Sending of heartbeat stopped'.format(nodeName = rospy.get_name()))

        # Disconnecting from IoT hub
        rospy.loginfo('{nodeName} : Disconnecting from IoT hub...'.format(nodeName = rospy.get_name()))
        self._device_client.disconnect()
        rospy.loginfo('{nodeName} : Disconnected from IoT hub'.format(nodeName = rospy.get_name()))

        # The IoT DeviceClient needs to be shutted down before destruction for graceful exit
        rospy.loginfo('{nodeName} : Shuting down IoT device client...'.format(nodeName = rospy.get_name()))
        self._device_client.shutdown()
        rospy.loginfo('{nodeName} : Shutted down IoT device client'.format(nodeName =rospy.get_name()))

    def send_heartbeat(self, _):
        """Sends D2C message containing if a ROSMaster is online and, therefore, if the robot is alive
        """
        if(rosgraph.is_master_online()):
            msg = Message(json.dumps({'isUp':1}))
        else:
            msg = Message(json.dumps({'isUp':0}))
        
        msg.message_id = uuid.uuid4()
        msg.correlation_id = 'leeloo-' + str(msg.message_id)
        msg.content_encoding = 'utf-8'
        msg.content_type = "application/json"

        rospy.loginfo('{nodeName} : Sending message'.format(nodeName = rospy.get_name()) + str(msg) + '...')
        self._device_client.send_message(msg)
        rospy.loginfo('{nodeName} : Message sent'.format(nodeName = rospy.get_name()))

if __name__ == "__main__":
    parser  = argparse.ArgumentParser(description="send heartbeat every [--period, -p] seconds")
    parser.add_argument("-p", "--period", help="period in second between heartbeat", type=int, default=10)
    # Apply arguments specific to ROS, returns the remaining arguments
    argv = rospy.myargv(sys.argv)
    # Parse arguments, first argument is ignore as it is the name of the script
    args=parser.parse_args(argv[1:])

    try:
        rospy.loginfo('{nodeName} : Connecting to ROSMaster...'.format(nodeName = rospy.get_name()))
        rospy.init_node('azure_heartbeat', anonymous=False)
        rospy.loginfo('{nodeName} : Connected to ROSMaster'.format(nodeName = rospy.get_name()))

        node = AzureHeartbeat(args.period)

        rospy.loginfo('{nodeName} : Heartbeat running'.format(nodeName = rospy.get_name()))
        rospy.spin()
        rospy.loginfo('{nodeName} : Shuting down Heartbeat'.format(nodeName = rospy.get_name()))
    
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())