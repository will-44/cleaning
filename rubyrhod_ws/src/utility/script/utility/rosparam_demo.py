#!/usr/bin/env python3

# Standard python submodules
import argparse
import sys
import traceback
from typing import List

# ROS imports
import rospy

class rosparamDemo:
    """Prints the values of the specified ros params every [period0] seconds
    """

    # Constructor of class, save list of rosparams to print
    # starts timer for printing rosparams
    def __init__(self, rosparams_list: List[str], period: int):
        """Prints the values of the specified ros params every [period] seconds

        :param rosparams_list: list of every rosparms to print in this instance
        :type rosparams_list: List[str]
        :param period: time interval between each time rosparams are printed
        :type period: int
        """

        rospy.on_shutdown(self._shutdown)

        # saving list of rosparams to print
        self._rosparams_list = rosparams_list

        # printing the rosparams every [period] seconds
        self._printing_timer = rospy.Timer(rospy.Duration(period), self.printing_rosparams)

    # Acts as a destructor, linked to the ropsy.on_shutdown method, will be invoked when the node is getting shut down
    def _shutdown(self):
        """stops timer to prevent accessing ressources as they are getting deallocated
        """

        self._printing_timer.shutdown()

    def printing_rosparams(self, _):
        """printing every rosparams in a single line
        """

        print_string: str = f'{rospy.get_name()}:'

        for rosparam in self._rosparams_list:
            print_string = print_string + f'\t{rosparam}: {rospy.get_param(rosparam, default="non-initialized")}'

        print(print_string)

# Script entry point if invoked as executable
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="prints specifieds rosparams [rosparams] every [--period, -p] seconds")
    parser.add_argument("rosparams", type=str, nargs='+')
    parser.add_argument("-p", "--period", help="period in second between telemetry messages", type=int, default=10)

    # Apply arguments specific to ROS, returns the remaining arguments
    argv = rospy.myargv(sys.argv)
    # Parse arguments, first argument is ignore as it is the name of the script
    args=parser.parse_args(argv[1:])

    nodeName: str ='rosparams_demo'

    for param in args.rosparams:
        nodeName = nodeName + f'_{param}'

    try:
        rospy.init_node(nodeName, anonymous=False)
        rospy.loginfo('{nodeName} : Connected to ROSMaster'.format(nodeName = rospy.get_name()))

        node = rosparamDemo(args.rosparams, args.period)

        rospy.loginfo('{nodeName} : Rosparam demo running'.format(nodeName = rospy.get_name()))
        rospy.spin()
        rospy.loginfo('{nodeName} : Shuting down rosparam demo'.format(nodeName = rospy.get_name()))
    
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())