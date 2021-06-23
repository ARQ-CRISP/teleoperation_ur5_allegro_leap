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
from teleoperation_ur5_allegro_leap.teleop.ur5 import WS_Bounds


def update_workspace(connection):
    global workspace_bounds
    workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=workspace_bounds)
    connection.workspace_marker.pose.position.x = workspace_bounds['center'][0]
    connection.workspace_marker.pose.position.y = workspace_bounds['center'][1]
    connection.workspace_marker.pose.position.z = workspace_bounds['center'][2]
    
    connection.workspace_marker.scale.x = workspace_bounds['scale'][0]
    connection.workspace_marker.scale.y = workspace_bounds['scale'][1]
    connection.workspace_marker.scale.z = workspace_bounds['scale'][2]  

if __name__ == '__main__':

    rospy.init_node('leap_2ur5')
    workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=None)
    
    if workspace_bounds is None:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' No Workspace Bounds set')
        workspace_bounds = {'center': [0.0, 0.4, 1.280], 'scale': [0.2] * 3}
        
        # workspace = WS_Bounds.from_center_scale([0.242, 0.364, 1.270], [0.2] * 3)
        rospy.set_param('ur5_teleop_config/workspace/', workspace_bounds)
    else:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' workspace_config: {}'.format(workspace_bounds))
    
    workspace = WS_Bounds.from_center_scale(workspace_bounds['center'], workspace_bounds['scale'])
    ur_controller = Leap_Teleop_UR5(workspace)
    # rospy.Timer(rospy.Duration(1/30.), lambda x: ur_controller.show_workspace())
    rospy.spin()