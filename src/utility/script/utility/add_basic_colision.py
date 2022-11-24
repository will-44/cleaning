#!/usr/bin/env python3

import rospy
from doosan import Doosan



if __name__ == '__main__':
    # open the doosan class to set the colissions
    rospy.init_node('add_basic_colision', anonymous=True)
    arm = Doosan()