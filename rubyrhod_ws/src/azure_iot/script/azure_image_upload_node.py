#!/usr/bin/env python3

# Standard python submodules
from cgitb import strong
import glob
from sre_constants import SUCCESS
import subprocess
import os
from pathlib import Path
import json
from typing import final
from unittest import result

# ROS imports
import rospy
import rospkg

# Azure IoT device import
import os
from azure.iot.device import IoTHubDeviceClient
from azure.iot.device.exceptions import ConnectionDroppedError, ConnectionFailedError, OperationTimeout, CredentialError, ClientError
from azure.core.exceptions import AzureError
from azure.storage.blob import BlobClient

# Global variable holding the path to the folder where rosbags will be stored
_image_folder=str(Path.home()) + "/image/"
#_image_folder="/media/USBDrive" + "/image/"
# Global variable holding the period between upload attempts
_upload_period=rospy.Duration(60)

# Global variable holding path to temporary file holding storage information from current upload for recovery
_tmp_storage_info=rospkg.RosPack().get_path("behaviors") + "/tmp/img_storage_info.json"

# Device client to connect to IoTHub
#_device_client=0
_device_client = IoTHubDeviceClient.create_from_connection_string(os.getenv('IOTHUB_DEVICE_CONNECTION_STRING'))

# def store_blob(file_path):
#     """Function to send file to blob storage connected to IoT Hub

#     :param file_path: path to file to be uploaded
#     :type file_path: path-like string
#     :return: did upload succeded?
#     :rtype: boolean
#     """

#     global _device_client

#     # hold if upload was successful, assume it was until error
#     success = True

#     try:
#         # Connect the client
#         _device_client.connect()
#         _device_client.connect()

#         # Get the storage info for the blob
#         blob_name = os.path.basename(file_path)
#         storage_info = _device_client.get_storage_info_for_blob(blob_name)
#         save_storage_info(storage_info)
    
#     except CredentialError as ex:
#         # Catch connection errors due to erroneous credentials
#         rospy.loginfo("Connection failed. Exception is: " + str(ex))
#         return False

#     except (ClientError, OperationTimeout) as ex:
#         # catch connection errors
#         rospy.loginfo("Connection failed. Exception is: " + str(ex))
#         return False
        
#     try:
#         # The SAS url contaions a one-time token to upload a file to the blob storage
#         sas_url = "https://{}/{}/{}{}".format(
#             storage_info["hostName"],
#             storage_info["containerName"],
#             storage_info["blobName"],
#             storage_info["sasToken"]
#         )

#         rospy.loginfo("\nUploading file: {} to Azure Storage as blob: {} in container {}\n".format(file_path, storage_info["blobName"], storage_info["containerName"]))

#         # Upload the specified file
#         with BlobClient.from_blob_url(sas_url) as blob_client:
#             with open(file_path, "rb") as f:
#                 result = blob_client.upload_blob(f, overwrite=True)

#                 # sent notification to IoTHub
#                 _device_client.notify_blob_upload_status(storage_info["correlationId"], True, 200, "OK: {}".format(file_path))

#                 rospy.loginfo("Upload succeeded. Result is: " + str(result))

#     except FileNotFoundError as ex:
#         # catch file not found and add an HTTP status code to return in notification to IoT Hub
#         ex.status_code = 404

#         _device_client.notify_blob_upload_status(storage_info["correlationId"], False, result.status_code, str(result))

#         rospy.loginfo("Upload failed. Exception is: " + str(ex))

#         success = False

#     except (ClientError, OperationTimeout) as ex:
#         # catch connection errors

#         _device_client.notify_blob_upload_status(storage_info["correlationId"], False, result.status_code, str(result))

#         rospy.loginfo("Connection failed. Exception is: " + str(ex))

#         success = False

#     except AzureError as ex:
#         # catch Azure errors that might result from the upload operation

#         _device_client.notify_blob_upload_status(storage_info["correlationId"], False, result.status_code, str(result))

#         rospy.loginfo("Upload failed. Exception is: " + str(ex))

#         success = False

#     finally:
#         _device_client.disconnect()
#         return success

def store_blob(file_path):
    """Function to send file to blob storage connected to IoT Hub

    :param file_path: path to file to be uploaded
    :type file_path: path-like string
    :return: did upload succeded?
    :rtype: boolean
    """

    global _device_client

    # hold if upload was successful, assume it was until error
    success = False

    # hold the state od the FST
    state = "Not_connected"

    # hold encountered errors
    connection_error = False
    credential_error = False

    # FST runs until reached end or stopping point
    while not rospy.is_shutdown():
        if state == "Not_connected":
            try:
                # Initialize FST registers
                connection_error = False
                credential_error = False

                # Attempt to connect
                _device_client.connect()

            except CredentialError as ex:
                rospy.loginfo("Connection failed. Exception is : " + str(ex))
                credential_error = True

            except (ConnectionFailedError, ConnectionDroppedError, OperationTimeout, ClientError) as ex:
                rospy.loginfo("Connection failed. Exception is : " + str(ex))
                connection_error = True

            finally:
                state = "Connection_attempt"

        elif state == "Connection_attempt":
            if credential_error:
                break
            elif connection_error:
                rospy.sleep(_upload_period)
                state = "Not_connected"
            else:
                # Initialize FST registers
                connection_error = False
                credential_error = False

                try:
                    # Get the storage info for the blob
                    blob_name = "image/" + os.path.basename(file_path)
                    storage_info = _device_client.get_storage_info_for_blob(blob_name)

                except CredentialError as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    credential_error = True

                except (ConnectionFailedError, ConnectionDroppedError, OperationTimeout, ClientError) as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    connection_error = True

                finally:
                    state = "Storage_info"

        elif state == "Storage_info":
            if credential_error:
                break
            elif connection_error:
                rospy.sleep(_upload_period)
                state = "Not_connected"
            else:
                # Initialize FST registers
                connection_error = False
                credential_error = False

                try:
                    save_storage_info(storage_info, success)

                    # The SAS url contaions a one-time token to upload a file to the blob storage
                    sas_url = "https://{}/{}/{}{}".format(
                        storage_info["hostName"],
                        storage_info["containerName"],
                        storage_info["blobName"],
                        storage_info["sasToken"]
                    )

                    # Upload the specified file
                    with BlobClient.from_blob_url(sas_url) as blob_client:
                        with open(file_path, "rb") as f:
                            result = blob_client.upload_blob(f, overwrite=True)
                            success = True
                            save_storage_info(storage_info, success)

                except CredentialError as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    credential_error = True
                
                except (ConnectionFailedError, ConnectionDroppedError, OperationTimeout, ClientError, AzureError) as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    connection_error = True

                finally:
                    state = "Upload"
        
        elif state == "Upload":
            if connection_error and not credential_error:
                rospy.sleep(_upload_period)
                state = "Retry_upload"
            else:
                # Initialize FST registers
                connection_error = False
                credential_error = False

                try:
                    _device_client.notify_blob_upload_status(storage_info["correlationId"], success,
                    200 if success else result.status_code, "OK: {}".format(file_path) if success else str(result))

                except CredentialError as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    credential_error = True

                except(ConnectionFailedError, ConnectionDroppedError, OperationTimeout, ClientError) as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    connection_error = True

                finally:
                    state = "Notification"

        elif state == "Retry_upload":
            try:
                # Initialize FST registers
                connection_error = False
                credential_error = False

                # Attempt to connect
                _device_client.connect()

            except CredentialError as ex:
                rospy.loginfo("Connection failed. Exception is : " + str(ex))
                credential_error = True

            except (ConnectionFailedError, ConnectionDroppedError, OperationTimeout, ClientError) as ex:
                rospy.loginfo("Connection failed. Exception is : " + str(ex))
                connection_error = True

            finally:
                state = "Reconnect_upload"
        
        elif state == "Reconnect_upload":
            if credential_error:
                break
            elif connection_error:
                rospy.sleep(_upload_period)
                state = "Retry_upload"
            else:
                try:
                    # Initialize FST registers
                    connection_error = False
                    credential_error = False

                    # Upload the specified file
                    with BlobClient.from_blob_url(sas_url) as blob_client:
                        with open(file_path, "rb") as f:
                            result = blob_client.upload_blob(f, overwrite=True)
                            success = True
                            save_storage_info(storage_info, success)

                except CredentialError as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    credential_error = True
                
                except (ConnectionFailedError, ConnectionDroppedError, OperationTimeout, ClientError) as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    connection_error = True

                finally:
                    state = "Upload"
        
        elif state == "Notification":
            if connection_error and not credential_error:
                rospy.sleep(_upload_period)
                state = "Retry_notification"
            else :
                if success:
                    delete_storage_info()
                
                break
        
        elif state == "Retry_notification":
            try:
                # Initialize FST registers
                connection_error = False
                credential_error = False

                # Attempt to connect
                _device_client.connect()

            except CredentialError as ex:
                rospy.loginfo("Connection failed. Exception is : " + str(ex))
                credential_error = True

            except (ConnectionFailedError, ConnectionDroppedError, OperationTimeout, ClientError) as ex:
                rospy.loginfo("Connection failed. Exception is : " + str(ex))
                connection_error = True

            finally:
                state = "Reconnect_notification"
        
        elif state == "Reconnect_notification":
            if credential_error:
                break
            elif connection_error:
                rospy.sleep(_upload_period)
                state = "Retry_upload"
            else:
                try:
                    # Initialize FST registers
                    connection_error = False
                    credential_error = False

                    _device_client.notify_blob_upload_status(storage_info["correlationId"], success,
                    200 if success else result.status_code, "OK: {}".format(file_path) if success else str(result))

                except CredentialError as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    credential_error = True
                
                except (ConnectionFailedError, ConnectionDroppedError, OperationTimeout, ClientError) as ex:
                    rospy.loginfo("Connection failed. Exception is : " + str(ex))
                    connection_error = True

                finally:
                    state = "Notification"

    _device_client.disconnect()
    return success

def azure_image_upload_node():
    """ROS node function
        
        initialize node,
        subscribes to topic /begin_write and link callback function to add the recently closed rosbag to the list of file to upload and memorise newly created rosback,
        subscribe to topic /log_node/ack and link callback function to end upload of rosbags,

        create client to IoTHub using connection string saved as environment variable
        Connect client

        spins
    """

    global _device_client
    global _upload_period

    # initializing node
    rospy.init_node('azure_image_upload_node', anonymous=False)

    # Creating device_clinet object using connection string saved in environment variables
    #rospy.loginfo(os.getenv('IOTHUB_DEVICE_CONNECTION_STRING'))
    #_device_client = IoTHubDeviceClient.create_from_connection_string(os.getenv('IOTHUB_DEVICE_CONNECTION_STRING'))

    # Hooking up shutdown function
    rospy.on_shutdown(shutdown_node)

    #rospy.Timer(_upload_period, upload_closed_rosbag_to_cloud, True)

    while not rospy.is_shutdown():
        upload_closed_rosbag_to_cloud()
        rospy.sleep(_upload_period)

    # Periodicly upload completed bags until killed or ROSCore stops
    rospy.spin()

def upload_closed_rosbag_to_cloud(event=None):
    global _image_folder

    os.chdir(_image_folder)
    
    files = glob.glob("*.jpg")
    for file in files:
    #for file in glob.glob(".jpg"):
        if store_blob(_image_folder + file) :
            os.remove(_image_folder + file)

def shutdown_node():
    global _device_client
    _device_client.shutdown()

def save_storage_info(storage_info, success):
    """Saves storage info to drive in temporary file in case of program shutdown

    :param storage_info: storage information received from IoTHub
    :type storage_info: _type_
    """

    storage_info["success"] = success
    json.dump(storage_info, open(_tmp_storage_info, "w+"))

def load_storage_info():
    """Imports storage information from the last started upload

    :return: storage info
    :rtype: dictionary
    """
    storage_info = json.load(open(_tmp_storage_info, "r"))
    success = storage_info["success"]
    storage_info.pop("success", None)

    return storage_info, success

def delete_storage_info():
    os.remove(_tmp_storage_info)

if __name__ == "__main__":
    azure_image_upload_node()
