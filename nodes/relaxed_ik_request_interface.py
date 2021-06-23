#! /usr/bin/env python
from __future__ import print_function, division, absolute_import
import numpy as np

import yaml
import rospy
from rospkg import RosPack
from teleoperation_ur5_allegro_leap.teleop.ur5.relaxed_ik_connection import Relaxed_UR5_Connection 
from teleoperation_ur5_allegro_leap.teleop.ur5 import WS_Bounds
from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix


# def update_workspace(connection):
#     global workspace_bounds
#     workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=workspace_bounds)
#     connection.workspace_marker.pose.position.x = workspace_bounds['center'][0]
#     connection.workspace_marker.pose.position.y = workspace_bounds['center'][1]
#     connection.workspace_marker.pose.position.z = workspace_bounds['center'][2]
    
#     connection.workspace_marker.scale.x = workspace_bounds['scale'][0]
#     connection.workspace_marker.scale.y = workspace_bounds['scale'][1]
#     connection.workspace_marker.scale.z = workspace_bounds['scale'][2]  
    

if __name__ == '__main__':
    rospy.init_node('ur_relaxed_connector')
    relaxed_ik_path = RosPack().get_path('relaxed_ik')
    relaxed_yaml_filename = rospy.get_param('~relaxed_ik_yaml', default='ur5_allegro_info.yaml')
    workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=None)
    is_sim = rospy.get_param('~simulated', default=True)
    
    rospy.loginfo('[' + rospy.get_name() + ']' + ' ik config_file: {}'.format(relaxed_yaml_filename))
    
    # if workspace_bounds is None:
    #     rospy.loginfo('[' + rospy.get_name() + ']' + ' No Workspace Bounds set')
    #     workspace_bounds = {'center': [0.0, 0.4, 1.280], 'scale': [0.2] * 3}
        
    #     # workspace = WS_Bounds.from_center_scale([0.242, 0.364, 1.270], [0.2] * 3)
    #     rospy.set_param('ur5_teleop_config/workspace/', workspace_bounds)
    # else:
    #     rospy.loginfo('[' + rospy.get_name() + ']' + ' workspace_config: {}'.format(workspace_bounds))
    
    # workspace = WS_Bounds.from_center_scale(workspace_bounds['center'], workspace_bounds['scale'])
        
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
        # workspace=workspace,
        sim=is_sim, debug_mode=True)
    
    connection.listen()
    
    while not rospy.is_shutdown():
        # update_workspace(connection)
        connection.consume_buffer()
    # rospy.spin()