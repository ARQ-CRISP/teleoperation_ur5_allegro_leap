#! /usr/bin/env python
from __future__ import print_function, absolute_import, division
from copy import deepcopy

import rospy
import numpy as np
from interactive_markers.interactive_marker_server import InteractiveMarker, InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, Pose
import PyKDL as kdl
from tf_conversions import posemath as pm
from teleoperation_ur5_allegro_leap.srv import Arm_Cartesian_Target
from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix
from teleoperation_ur5_allegro_leap.teleop.ur5 import WS_Bounds
from teleoperation_ur5_allegro_leap.teleop.ur5 import InteractiveControl
 
class InteractiveControlNode(object):
    pose_goal_topic = ur5_teleop_prefix + 'arm_pose_targets'
    def __init__(self, marker_init_pos=None, marker_init_quat=None, workspace=None):
        
        self.interactive_control = InteractiveControl(marker_init_pos, marker_init_quat)
        
        self.workspace = workspace
        rospy.wait_for_service(self.pose_goal_topic)
        self.send_target = rospy.ServiceProxy(self.pose_goal_topic, Arm_Cartesian_Target)
        # self.posegoal_pub = rospy.Publisher(self.pose_goal_topic, PoseStamped, queue_size=1)
        rospy.Timer(rospy.Duration(1/30.), lambda x: self.publish_pose())
        
    def publish_pose(self):
        # print('Publishing')
        frame = deepcopy(self.interactive_control.frame)
        if self.workspace is not None:
            frame.p = kdl.Vector(*self.workspace.bind(frame.p))
        target = PoseStamped()
        target.pose = pm.toMsg(frame)
        target.header.frame_id = 'world'
        # self.posegoal_pub.publish(target)
        self.send_target(absolute=True, query=target)
 
 
if __name__ == '__main__':
    rospy.init_node('UR5_ee_control_marker')
    workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=None)
    
    if workspace_bounds is None:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' No Workspace Bounds set')
        workspace_bounds = {'center': [0.0, 0.4, 1.280], 'scale': [0.2] * 3}
        # workspace = WS_Bounds.from_center_scale([0.242, 0.364, 1.270], [0.2] * 3)
        rospy.set_param('ur5_teleop_config/workspace/', workspace_bounds)
    else:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' workspace_config: {}'.format(workspace_bounds))
    
    workspace = WS_Bounds.from_center_scale(workspace_bounds['center'], workspace_bounds['scale'])
    controller = InteractiveControlNode([0.200, 0.358, 1.147], [0.708, 0.000, -0.707, 0.000], workspace)
    rospy.spin()