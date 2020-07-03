#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 15/11/19
'''
from __future__ import division, print_function
# from pynput.keyboard import Key, Controller, Listener, KeyCode
from rospkg.rospack import RosPack
from pickle import load
import numpy as np
import rospy
import tf2_ros
from teleoperation_ur5_allegro_leap import Leap_Teleop_Allegro
from teleoperation_ur5_allegro_leap import EventCatcher
from teleoperation_ur5_allegro_leap.teleop.allegro.calibration import listoffingers_to_dict, set_calibration_pose_param, get_max_pose_index
from teleoperation_ur5_allegro_leap.teleop.app.calibration_gui import Calibration_GUI



if __name__ == "__main__":

    rospy.init_node('leap2allegro')
    tf_buffer = tf2_ros.Buffer(rospy.Duration(50))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    lefthand_mode = rospy.get_param('~left_hand', False)
    leap_topic = rospy.get_param('~leap_topic', '/leap_motion/leap_filtered')
    package_path = RosPack().get_path('relaxed_leap_teleop')

    calibration_file = package_path + '/config/calibration_allegro_states.pkl'
    with open(calibration_file, 'r') as ff:
        calibration_poses = load(ff)
        calibration_poses = dict([(key, listoffingers_to_dict(
            finger)) for key, finger in calibration_poses.items()])
    # keyboard = Controller()

    set_calibration_pose_param(
        calibration_poses, Leap_Teleop_Allegro.pose_param)
    allegro_teleop = Leap_Teleop_Allegro(
        tf_buffer, leap_topic, lefthand_mode, scale=[1.0, 1.0, 1.0, 1.0])
    rospy.on_shutdown(allegro_teleop.on_shutdown)
    # stop_calibration = False

    ec = EventCatcher()
    calib_gui = Calibration_GUI(allegro_teleop, ec)
    ec.show_keybinders()

    ec.mainloop()
