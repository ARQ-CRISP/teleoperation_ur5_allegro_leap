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

class InteractiveControl(object):
    
    def __init__(self, marker_init_pos=None, marker_init_quat=None, marker_name="UR_EE_Marker"):
        if marker_init_pos is None:
            marker_init_pos = [0.]*3
        if marker_init_quat is None:
            marker_init_quat = ([0.]*3)+[1.]
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
        self.int_mark.description = marker_name
        
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

    def makeMarker(self, msg, marker_type=Marker.ARROW, scale = 0.145):
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
        control.markers.append(self.makeMarker(msg))
        msg.controls.append(control)
        return control
    
    def processFeedback(self, feedback):
        frame = pm.fromMsg(feedback.pose)
        self.frame = frame
        self.server.setPose(feedback.marker_name, feedback.pose)
        self.server.applyChanges()
        print(
            feedback.marker_name + " is now at ",
            np.asarray(list(frame.p)).round(3),
            np.asarray(list(frame.M.GetQuaternion())).round(3))
       
