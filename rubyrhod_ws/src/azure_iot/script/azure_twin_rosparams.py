#!/usr/bin/env python3

# Standard python submodules
import argparse
import os
import sys
import traceback
from typing import List

# ROS imports
import rospy

# Azure IoT device import
from azure.iot.device import IoTHubModuleClient, IoTHubDeviceClient

class AzureTwinRosparams:
    """This node recieves twin desired properties and applies them to rosparams
    """

    # Construtor of class, initiates IoT Hub connection and connect callback functions
    # to desired properties updates
    def __init__(self) -> None:
        """This node recieves twin desired properties and applies them to rosparams
        """

        rospy.on_shutdown(self.__shutdown)

        # Obtaning security type
        security_type = os.getenv("IOTHUB_DEVICE_SECURITY_TYPE")

        if security_type == "connectionString":
            self._module_client = IoTHubModuleClient.create_from_connection_string(os.getenv("IOTHUB_DEVICE_CONNECTION_STRING"))
        
        elif security_type == "edgeEnvironment":
            self._module_client = IoTHubModuleClient.create_from_edge_environment()

        else:
            raise RuntimeError(
                "A connection security type must be selected in IOTHUB_DEVICE_SECURITY_TYPE"
            )

        self._module_client.on_twin_desired_properties_patch_received = \
                self.twin_desired_properties_patch_handler
        
        # Connect the client.
        self._module_client.connect()
        
    def __shutdown(self):
        self._module_client.shutdown()

    def twin_desired_properties_patch_handler(self, patch):
        if "rosparams" in patch:
            self.apply_twin_desired_rosparams(patch["rosparams"])

    def apply_twin_desired_rosparams(self, desired_rosparams:dict):
        for desired_rosparam_key, desired_rosparam_value in desired_rosparams.items():
            if desired_rosparam_value is not None:
                rospy.set_param(desired_rosparam_key, desired_rosparam_value)
            
            else:
                rospy.delete_param(desired_rosparam_key)

        self._module_client.patch_twin_reported_properties({"rosparams": desired_rosparams})

# Script entry point if invoked as executable
if __name__ == "__main__":
    # Apply arguments specific to ROS, returns the remaining arguments
    argv = rospy.myargv(sys.argv)

    try:
        rospy.init_node('azure_twin_to_rosparams', anonymous=False)
        rospy.loginfo('{nodeName} : Connected to ROSMaster'.format(nodeName = rospy.get_name()))

        node = AzureTwinRosparams()

        rospy.loginfo('{nodeName} : Device twin to rosparam running'.format(nodeName = rospy.get_name()))
        rospy.spin()
        rospy.loginfo('{nodeName} : Shuting down device twin to rosparam'.format(nodeName = rospy.get_name()))
    
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())