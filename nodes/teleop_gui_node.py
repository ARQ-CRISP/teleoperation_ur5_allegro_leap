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
from teleoperation_ur5_allegro_leap import EventCatcher, Calibration_GUI, Movegroup_GUI, Experiments_GUI
from teleoperation_ur5_allegro_leap.teleop.allegro.calibration import listoffingers_to_dict, set_calibration_pose_param, get_max_pose_index
# from teleoperation_ur5_allegro_leap.teleop.app.calibration_gui import Calibration_GUI
# from teleoperation_ur5_allegro_leap.teleop.app.ur5_moveit_gui import Movegroup_GUI
# from teleoperation_ur5_allegro_leap import Cali


if __name__ == "__main__":

    rospy.init_node('teleop_gui')
    tf_buffer = tf2_ros.Buffer(rospy.Duration(50))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    lefthand_mode = rospy.get_param('~left_hand', False)
    leap_topic = rospy.get_param('~leap_topic', '/leap_motion/leap_filtered')
    package_path = RosPack().get_path('teleoperation_ur5_allegro_leap')

    calibration_file = package_path + '/config/calibration_allegro_states.pkl'
    with open(calibration_file, 'r') as ff:
        calibration_poses = load(ff)
        calibration_poses = dict([(key, listoffingers_to_dict(
            finger)) for key, finger in calibration_poses.items()])


    ec = EventCatcher(size=(300, 600))
    rospy.on_shutdown(ec.close_app)
    ec.set_status('Execution_Mode')
    ec.set_title('Teleop Controller')
    calib_gui = Calibration_GUI(ec)
    movegrp_gui = Movegroup_GUI('ur5_arm', event_catcher=ec, step=0.05)
    experiment_gui = Experiments_GUI(movegrp_gui, event_catcher=ec)
    ec.show_keybinders()

    ec.mainloop()
