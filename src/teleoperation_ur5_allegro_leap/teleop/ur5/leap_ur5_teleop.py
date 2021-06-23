#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 2/06/21
'''
from __future__ import print_function, division, absolute_import

import tf2_ros
import rospy
import numpy as np
from PyKDL import Frame, Rotation, Vector

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from leap_motion.msg import Human
from tf_conversions import fromTf, toMsg

from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix

from teleoperation_ur5_allegro_leap.srv import Toggle_Tracking, Toggle_TrackingResponse

def transform_to_vec(T):
    t = T.transform.translation
    r = T.transform.rotation
    return [[t.x, t.y, t.z], [r.x, r.y, r.z, r.w]]

class Leap_Teleop_UR5():
    
    leap_motion_topic = '/leap_motion/leap_device'
    marker_topic = ur5_teleop_prefix + 'target_marker'
    pose_goal_topic = ur5_teleop_prefix + 'ur5_pose_targets'
    toggle_tracking_srv = ur5_teleop_prefix + 'toggle_tracking'
    workspace_marker_topic = ur5_teleop_prefix + 'workspace'
    
    def __init__(self, workspace, right_hand=True):
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.previous_tf = None
        self.workspace = workspace
        self.create_ws_viz_properties()
        
        self.target = Frame()
        self.right_hand_mode = right_hand
        rospy.sleep(.1) #Let the tf buffer fill up
        self.bias = Rotation()#Frame(Rotation.Quaternion(*[0.0, -0.7071067811865475, 0.7071067811865476, 0.0]), Vector())
        self.current_pose = fromTf(transform_to_vec(self.tf_buffer.lookup_transform(
            'world', 'hand_root', rospy.Time(0), rospy.Duration(0.1))))
        
        self.target_marker = Marker()
        self.target_marker.type = Marker.ARROW
        self.target_marker.scale.x, self.target_marker.scale.y, self.target_marker.scale.z = 0.02, 0.01, 0.01
        self.target_marker.color.r = self.target_marker.color.a = 1.0
        self.target_marker.header.frame_id = 'world'
        self.__leap_listener = rospy.Subscriber(self.leap_motion_topic, Human, self.OnLeapMessage, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1)
        self.posegoal_pub = rospy.Publisher(self.pose_goal_topic, PoseStamped, queue_size=1)
        
        self.__tracking_toggler = rospy.Service(self.toggle_tracking_srv, Toggle_Tracking,
                                                lambda update: Toggle_TrackingResponse( 
                                                    self.toggle_tracking() if update.update else self.is_tracking))
    
    def create_ws_viz_properties(self):
        self.marker_workspace_pub = rospy.Publisher(self.workspace_marker_topic, Marker, queue_size=1)
        self.workspace_marker = Marker(type=Marker.CUBE)
        ws_center, ws_scale = self.workspace.get_center_scale()
        # print(ws_center, ws_scale)
        self.workspace_marker.header.frame_id ='world'
        self.workspace_marker.pose.position.x, self.workspace_marker.pose.position.y, self.workspace_marker.pose.position.z = ws_center
        self.workspace_marker.pose.orientation.w = 1.
        self.workspace_marker.scale.x, self.workspace_marker.scale.y, self.workspace_marker.scale.z = ws_scale
        self.workspace_marker.color.g = self.workspace_marker.color.a = .5
        
        self.timer = rospy.Timer(rospy.Duration(1/30.), lambda x: self.show_workspace())
        # self.timer.start()
        
        
    def show_workspace(self):
        self.marker_workspace_pub.publish(self.workspace_marker)
        
    def toggle_tracking(self):
        if self.__leap_listener is None:
            rospy.loginfo('UR5 Teleop: Resuming Tracking!')
            self.__leap_listener = rospy.Subscriber(
                self.leap_motion_topic, Human, self.OnLeapMessage, queue_size=1)
        else:
            rospy.loginfo('UR5 Teleop: Tracking Interrupted!')
            self.__leap_listener.unregister()
            self.__leap_listener = None
        return self.is_tracking    
            
    @property         
    def is_tracking(self):
        return self.__leap_listener is None              
             
             
    def OnLeapMessage(self, human_msg):
        
        if self.right_hand_mode:
            leap_hand = human_msg.right_hand
        else:
            leap_hand = human_msg.left_hand
            
        if leap_hand.is_present:
            leap2wrist = self.tf_buffer.lookup_transform(
                'world', 'right_leap_hand_wrist', rospy.Time(0), rospy.Duration(0.1))
            leap2wrist
            wrist_f = fromTf(transform_to_vec(leap2wrist))
            
            if self.previous_tf is not None:
                
                self.target.p = wrist_f.p - self.previous_tf.p
                self.target.M = wrist_f.M

                # bb = Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0 ,0)
                # tgt = Frame(Rotation.Quaternion(0.5, 0.5, 0.5, -0.5)) * Frame(bb) * self.target * Frame(bb).Inverse()
                self.current_pose.p += self.target.p
                self.current_pose.M = self.target.M
                # self.current_pose.p += tgt.p
                # self.current_pose.M = tgt.M
                bound_pos = self.workspace.bind(self.current_pose.p)
                self.current_pose.p = Vector(*bound_pos)
                rospy.loginfo(rospy.get_name() + ': ' + str(np.array(list(self.current_pose.p)).round(3)))
                self.target_marker.pose = toMsg(self.current_pose)
            
                self.marker_pub.publish(self.target_marker )
                stamped_target = PoseStamped()
                stamped_target.header.frame_id = 'world'
                stamped_target.pose = toMsg(self.current_pose)
                self.posegoal_pub.publish(stamped_target)
            
            self.previous_tf = wrist_f