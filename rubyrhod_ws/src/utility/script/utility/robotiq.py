#!/usr/bin/env python3

import rospy

from robotiq_85_msgs.msg import GripperCmd, GripperStat

class Robotiq:
    '''
    This class communicate with the robotiq gripper's topics.
    Before using the gripper, you need to run the robotiq_85_driver script.
    '''

    def __init__(self): 
        # Initialize the publisher
        self.robotiq_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)
        
        # Create the message variable
        self.command = GripperCmd()

        # Gripper's constant (use for the 2F-85)
        self.openPosition = 0.085
        self.closePosition = 0
        self.defaultSpeed = 0.02
        self.defaultForce = 100

        # Wait for the gripper to be ready
        # ready = False
        # while not (ready):
        #     ready = True
        #     ready &= self.status()
        # rospy.loginfo("Gripper ready")

    def noEmergency(self):
        """
        Make sure the gripper has no emergency
        """
        self.command.emergency_release = False
        self.command.emergency_release_dir = 1
        self.command.stop = False

    def custom_order(self, position, speed = None, force = None):
        """
        Set the gripper to a position.
        Speed and force will clamp at min and max value.

        :param position: desired position. Value between 0 (close) and 0.085 (open)
        :param speed: desired speed of the motion. Value between 0 (min) and .1 (max)
        :param force: desired force of the motion. Value between 0 (min) and 220 (max)
        """
        # Set speed and force if not parameter were given
        if speed == None:
            speed = self.defaultSpeed
        if force == None:
            force = self.defaultForce
        
        # Set message values
        self.command.position = position
        self.command.speed = speed
        self.command.force = force
        self.noEmergency()

        # Publish the command
        self.robotiq_pub.publish(self.command)
        

    def close(self, speed = None, force = None):
        """
        Send the gripper in its close position

        :param position: desired position
        :param speed: desired speed of the motion
        :param force: desired force of the motion
        """
        self.custom_order(self.closePosition, speed, force)

    def open(self, speed = None, force = None):
        """
        Send the gripper in its open position

        :param position: desired position
        :param speed: desired speed of the motion
        :param force: desired force of the motion
        """
        self.custom_order(self.openPosition, speed, force)
 
    def stop(self):
        """
        Send a command to stop the gripper
        """
        self.command.stop = True
        self.robotiq_pub(self.command)

    def status(self):
        """
        This will give get the current status of the gripper and
        send it to ROS.

        :return: Status of the gripper
        """
        msg = rospy.wait_for_message("/gripper/stat", GripperStat, timeout=1)
        return msg
    




