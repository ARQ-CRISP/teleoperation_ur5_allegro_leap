#! /usr/bin/env python
from __future__ import print_function, division, absolute_import
import numpy as np

import yaml
import rospy
from rospkg import RosPack
from teleoperation_ur5_allegro_leap.teleop.ur5.relaxed_ik_connection import Relaxed_UR5_Connection 
# from teleoperation_ur5_allegro_leap.teleop.ur5 import WS_Bounds
# from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix

    

if __name__ == '__main__':
    rospy.init_node('ur_relaxed_connector')
    relaxed_ik_path = RosPack().get_path('relaxed_ik')
    relaxed_yaml_filename = rospy.get_param('~relaxed_ik_yaml', default='ur5_allegro_info.yaml')
    workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=None)
    is_sim = rospy.get_param('~simulated', default=True)
    
    rospy.loginfo('[' + rospy.get_name() + ']' + ' ik config_file: {}'.format(relaxed_yaml_filename))
    
        
    if is_sim:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' Teleoperating in a simulated environment!')
    else:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' Teleoperating on the real robot!')
    yaml_file_path = relaxed_ik_path + '/src/' + '/RelaxedIK/Config/info_files/' + relaxed_yaml_filename
    with open(yaml_file_path) as f:
        yaml_file = yaml.load(f)


    connection = Relaxed_UR5_Connection(
        init_state=yaml_file['starting_config'],
        movegroup='ur5_arm',
        sim=is_sim, debug_mode=True)
    
    # connection.listen()
    rospy.on_shutdown(connection.on_kill)
    # while not rospy.is_shutdown():
        # connection.consume_buffer()
    rospy.spin()