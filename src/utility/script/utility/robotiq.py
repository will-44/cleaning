#!/usr/bin/env python3
'''
Request gripper:
rACT: 0 reset 1 activation
rATR: ?
rGTO: 0 
rSP: 0-255 speed
rFR: 0-255 force
rPR: 0-255 position

Status gripper:
gACT: 0 reset 1 activation
gGTO: 0 Standby 1 go to position request
gSTA: 0 in reset 
      1 activation in progress 
      2 not used 
      3 activation completed
gOBJ: 0 Fingers are in motion 
      1 Fingers have stopped due to a contact while opening 
      2 Fingers have stopped due to a contact while closing 
      3 Fingers are at requested position
gFLT: 0 No Fault
      1 Priority Fault: Action delayed, initialization must be completed prior to action
      2 Priority Fault: The activation bit must be set prior to action
      3 Minor Fault: The communication chip is not ready (may be booting)
      4 Minor Fault: Automatic release in progress
      5 Major Fault: Overcurrent protection triggered
      6 Major Fault: Automatic release completed
gPR: Echo of the requested position for the Gripper (0-255)
gPO: Position of Fingers (0-255)
gCU: Current of Fingers X*10 mA
'''

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

'''This class communicate with the robotiq gripper's topics
Before using the gripper, reset it and activate it'''
class Robotiq:
    def __init__(self):
        self.robotiq_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)

    def reset(self):
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0
        self.robotiq_pub.publish(command)

    def activate(self):
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rATR = 1
        command.rPR = 0
        command.rSP = 255
        command.rFR = 150
        self.robotiq_pub.publish(command)

    def close(self):
        '''
        This methode close the gripper
        '''
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150
        command.rPR = 255
        self.robotiq_pub.publish(command)

    def open(self):
        '''
        This methode open the gripper
        '''
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP = 255
        command.rFR = 150
        command.rPR = 0
        self.robotiq_pub.publish(command)

    def status(self):
        '''
        This methode open the gripper
        '''

        msg = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, timeout=10)
        return msg



