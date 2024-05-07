#!/usr/bin/env python3

# Standard python submodules
import subprocess
from datetime import datetime
from pathlib import Path
from sys import prefix

# ROS imports
import rospy
from std_msgs.msg import String

# Global variable holding the path to the folder where rosbags will be stored
#_bag_log_folder=str(Path.home()) + "/bag_logs/"
_bag_log_folder="/media/USBDrive" + "/bag_logs/"
# Global variable holding prefix for rosbag filename
_bag_prefix="session"
# Global variable holding the duration if each rosbags in a session
_bag_duration="10"

# Keeping if a recording is already running
_recording_running = False

def rosbag_log_node():
    """ROS node function
        
        initialize node,
        subscribes to topic /log_node/cmd,
        spins
    """

    # Initializing node
    rospy.init_node('rosbag_log_node', anonymous=False)

    # Subscribing to the command topic
    rospy.Subscriber("/log_node/cmd", String, callback_cmd)

    # Keep node alive until killed or ROSCore stops
    rospy.spin()

def callback_cmd(data):
    """Callback function used when node receives message on topic /log_node/cmd

        If message recieved is "log_start", call the start_log function
        If message recieved is "log_stop", call the stop_log function

    :param data: message recieved on topic
    :type data: std_msgs/String
    """
    try:
        {
            'log_start' : start_log,
            'log_stop' : stop_log,
        }[data.data]()
    except KeyError:
        pass

def start_log():
    """Starts the rosbag recording using bash command passed directly to the shell
        If a recording is already running, nothing happens
    """

    global _bag_prefix
    global _bag_log_folder
    global _bag_duration

    global _recording_running

    if not _recording_running:
        # Starting recording in a new shell
        subprocess.Popen("rosbag record --duration {duration} --split -o {prefix} -a __name:=log_node".format(duration=_bag_duration, prefix = _bag_prefix), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, cwd=_bag_log_folder)

        # Recording process has started
        _recording_running = True

def stop_log():
    """Stops rosbag recording using bash command passed directly to the shell
    """

    global _recording_running

    if _recording_running:
        # Stop recording via a rosnode kill of the created node
        subprocess.run("rosnode kill /log_node", shell=True)

        # Recording is done
        _recording_running = False

if __name__ == "__main__":
    rosbag_log_node()
