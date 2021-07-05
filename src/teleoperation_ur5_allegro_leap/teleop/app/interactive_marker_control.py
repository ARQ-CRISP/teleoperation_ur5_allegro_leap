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
from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix
from teleoperation_ur5_allegro_leap.teleop.ur5 import WS_Bounds

class InteractiveControl():
    pose_goal_topic = ur5_teleop_prefix + 'ur5_pose_targets'
    def __init__(self, marker_init_pos=None, marker_init_quat=None, workspace=None):
        if marker_init_pos is None:
            marker_init_pos = [0.]*3
        if marker_init_quat is None:
            marker_init_quat = ([0.]*3)+[1.]
        self.workspace = workspace
        self.frame = kdl.Frame(
                kdl.Rotation.Quaternion(*marker_init_quat),
                kdl.Vector(*marker_init_pos)
                )
        self.server = InteractiveMarkerServer('teleoperation_bias_marker')
        
        self.int_mark = InteractiveMarker()
        self.int_mark.header.frame_id = "world"
        self.int_mark.pose = pm.toMsg(self.frame)
        self.int_mark.scale = .5

        self.int_mark.name = "ur5_target_bias"
        self.int_mark.description = "UR_EE_Marker"
        
        self.makeMarkerControl(self.int_mark)

        control = InteractiveMarkerControl()
        control.orientation = pm.toMsg(kdl.Frame(kdl.Rotation.Quaternion(0., 0.707, 0., 0.707))).orientation
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_mark.controls.append(deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_mark.controls.append(deepcopy(control))
        control.orientation = pm.toMsg(kdl.Frame(kdl.Rotation.Quaternion(0., 0., 0., 1.))).orientation
        self.int_mark.controls.append(deepcopy(control))
        control.orientation = pm.toMsg(kdl.Frame(kdl.Rotation.Quaternion(0., 0., 0.707, 0.707))).orientation
        self.int_mark.controls.append(deepcopy(control))

        self.server.insert(self.int_mark, self.processFeedback)
        
        # 'commit' changes and send to all clients
        self.server.applyChanges()
        
        self.posegoal_pub = rospy.Publisher(self.pose_goal_topic, PoseStamped, queue_size=1)
        rospy.Timer(rospy.Duration(1/30.), lambda x: self.publish_pose())
    
    def makeMarker(self, msg, marker_type=Marker.CUBE, scale = 0.145):
        marker = Marker()

        marker.type = marker_type
        marker.scale.x = msg.scale * scale
        marker.scale.y = msg.scale * scale
        marker.scale.z = msg.scale * scale
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        
        return marker
        
    def makeMarkerControl(self, msg):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.makeMarker(msg, Marker.ARROW))
        msg.controls.append(control)
        return control

    def publish_pose(self):
        # print('Publishing')
        frame = deepcopy(self.frame)
        if self.workspace is not None:
            frame.p = kdl.Vector(*self.workspace.bind(frame.p))
        target = PoseStamped()
        target.pose = pm.toMsg(frame)
        target.header.frame_id = 'world'
        self.posegoal_pub.publish(target)

    
    def processFeedback(self, feedback):
        frame = pm.fromMsg(feedback.pose)
        self.frame = frame
        print(feedback.marker_name + " is now at ", np.asarray(list(frame.p)).round(3), np.asarray(list(frame.M.GetQuaternion())).round(3))
        
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
    controller = InteractiveControl([0.200, 0.358, 1.147], [0.708, 0.000, -0.707, 0.000], workspace)
    rospy.spin()