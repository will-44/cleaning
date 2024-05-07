#!/usr/bin/env python3

# Standard python submodules
import sys
import traceback
from typing import List
import subprocess

# ROS imports
import rospy
import rospkg

# Imports from the utility package
from utility.variable_timer import VariableTimer

class rosparamKinovaDemo:
    """Runs movement demos #[scriptToRun] from Kinova's SDK every [loopPeriod] seconds
    """

    # Constructor of class, starts timer for runing exemples and updating timer period
    def __init__(self):
        """Runs movement demos #[scriptToRun] from Kinova's SDK every [loopPeriod] seconds
        """

        rospy.on_shutdown(self.__shutdown)

        self._exemple_scripts_path = rospkg.RosPack().get_path('utility') \
            + '/dep/kortex/api_python/examples/102-Movement_high_level/'
        
        self._exemple_scripts_files = {
            1: '01-move_angular_and_cartesian.py',
            2: '02-sequence.py',
            3: '03-twist_command.py',
            4: '04-send_joint_speeds.py'
        }

        self._script_timer = VariableTimer( \
            rospy.Duration(rospy.get_param('loopPeriod', 10)), \
            self.run_scripts)

    def __shutdown(self):
        """stops timer to prevent accessing ressources as they are getting deallocated
        """

        self._script_timer.shutdown()

    def run_scripts(self, _):
        """invokes Kinova's movement exemple scripts
            also updates the timer's period
        """

        # Updates the timer's period if the desired period changed
        if (period := rospy.Duration(rospy.get_param('loopPeriod', 10))) != self._script_timer._period:
            self._script_timer.set_period(period)

        try:
            subprocess.run(args=['python3', self._exemple_scripts_path + \
                            self._exemple_scripts_files[rospy.get_param('scriptToRun')]])
        except KeyError:
            pass

# Script entry point if invoked as executable
if __name__ == "__main__":
    # Apply arguments specific to ROS, returns the remaining arguments
    argv = rospy.myargv(sys.argv)

    nodeName: str ='rosparams_kinova_demo'

    try:
        rospy.init_node(nodeName, anonymous=False)
        rospy.loginfo('{nodeName} : Connected to ROSMaster'.format(nodeName = rospy.get_name()))

        node = rosparamKinovaDemo()

        rospy.loginfo('{nodeName} : Rosparam demo running'.format(nodeName = rospy.get_name()))
        rospy.spin()
        rospy.loginfo('{nodeName} : Shuting down rosparam demo'.format(nodeName = rospy.get_name()))
    
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())