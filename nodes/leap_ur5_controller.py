#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 2/06/21
'''
from __future__ import print_function, division, absolute_import

import rospy
from teleoperation_ur5_allegro_leap.teleop.ur5 import Leap_Teleop_UR5


if __name__ == '__main__':

    rospy.init_node('leap_2ur5')
    
    ur_controller = Leap_Teleop_UR5()
    rospy.spin()