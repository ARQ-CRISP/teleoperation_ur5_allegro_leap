#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 2/06/21
'''
from __future__ import print_function, division, absolute_import

import numpy as np
from collections import deque
from enum import Enum

import rospy
import tf2_ros
from PyKDL import Frame, Rotation, Vector
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from leap_motion.msg import Human
from tf_conversions import fromTf, toMsg

from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix

from teleoperation_ur5_allegro_leap.srv import Toggle_Tracking, Toggle_TrackingResponse
from teleoperation_ur5_allegro_leap.srv import Toggle_ArmTeleopMode, Toggle_ArmTeleopModeResponse
from teleoperation_ur5_allegro_leap.srv import Arm_Cartesian_Target, Arm_Cartesian_TargetResponse
# from teleoperation_ur5_allegro_leap.srv import Go_To_Base, Go_To_BaseResponse

def transform_to_vec(T):
    t = T.transform.translation
    r = T.transform.rotation
    return [[t.x, t.y, t.z], [r.x, r.y, r.z, r.w]]

class Teleop_Mode(Enum):
    base_pos = 0
    marker_pos = 1
    
    def __str__(self):
        strings = ['Velocity - Base Pose', 'Velocity - Marker Pose']
        return strings[self.value]


class Leap_Teleop_UR5():
    
    leap_motion_topic = '/leap_motion/leap_device'
    marker_topic = ur5_teleop_prefix + 'target_marker'
    pose_goal_topic = ur5_teleop_prefix + 'arm_pose_targets'
    toggle_tracking_srv = ur5_teleop_prefix + 'toggle_tracking'
    toggle_teleop_mode_srv = ur5_teleop_prefix + 'toggle_teleop_mode'
    move_marker_srv = ur5_teleop_prefix + 'toggle_teleop_mode'
    # go_to_base_srv = ur5_teleop_prefix + 'go_to_base'
    workspace_marker_topic = ur5_teleop_prefix + 'workspace'
    
    def __init__(self, workspace, right_hand=True):
        
        self.combine_with_marker = True #TOREMOVE
        self.teleop_mode = Teleop_Mode.marker_pos
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.previous_tf = self.first_tf = None
        self.last_sent_target = None
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
        # self.posegoal_pub = rospy.Publisher(self.pose_goal_topic, PoseStamped, queue_size=1)
        rospy.wait_for_service(self.pose_goal_topic)
        self.send_target = rospy.ServiceProxy(self.pose_goal_topic, Arm_Cartesian_Target)
        self.__leap_listener = rospy.Subscriber(self.leap_motion_topic, Human, self.OnLeapMessage, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1)

        self.position_buffer = deque(maxlen=10)
        self.orientation_buffer = deque(maxlen=10)
        
        self.__tracking_toggler = rospy.Service(self.toggle_tracking_srv, Toggle_Tracking,
                                                lambda msg: Toggle_TrackingResponse( 
                                                    self.toggle_tracking() if msg.update else self.is_tracking))
        
        self.__teleop_mode_toggler = rospy.Service(self.toggle_teleop_mode_srv, Toggle_ArmTeleopMode,
                                                lambda msg: Toggle_ArmTeleopModeResponse(mode=self.toggle_teleop_mode(msg.update)))

    #     self.__go_to_base = rospy.Service(self.go_to_base_srv, Go_To_Base,
    #                                             lambda msg: Go_To_BaseResponse(result=self.go_to_base()))
    
    # def go_to_base(self):
    #     if self.teleop_mode == Teleop_Mode.marker_pos:
    #         self.target = Frame(Rotation.Quaternion(*[-0.707, -0.000, 0.707, -0.000]))
    #     elif self.teleop_mode == Teleop_Mode.base_pos:
    #         self.target = Frame()
    #     else:
    #         return False
    #     try:
    #         self.go_to_pose(self.target)
    #         return True
    #     except Exception as e:
    #         return False

    def create_ws_viz_properties(self):
        self.marker_workspace_pub = rospy.Publisher(self.workspace_marker_topic, Marker, queue_size=1)
        self.workspace_marker = Marker(type=Marker.CUBE)
        ws_center, ws_scale = self.workspace.get_center_scale()
        # print(ws_center, ws_scale)
        self.workspace_marker.header.frame_id ='world'
        self.workspace_marker.pose.position.x, self.workspace_marker.pose.position.y, self.workspace_marker.pose.position.z = ws_center
        self.workspace_marker.pose.orientation.w = 1.
        self.workspace_marker.scale.x, self.workspace_marker.scale.y, self.workspace_marker.scale.z = (np.asarray(ws_scale)*2).tolist()
        self.workspace_marker.color.g = self.workspace_marker.color.a = .5
        
        self.timer = rospy.Timer(rospy.Duration(1/30.), lambda x: self.show_workspace())
        # self.timer.start()
        
        
    def show_workspace(self):
        self.marker_workspace_pub.publish(self.workspace_marker)
        
    def toggle_tracking(self):
        if not self.is_tracking:
            rospy.loginfo('Arm Teleop: Resuming Tracking!')
            self.previous_tf = None
            self.__leap_listener = rospy.Subscriber(
                self.leap_motion_topic, Human, self.OnLeapMessage, queue_size=1)
        else:
            rospy.loginfo('Arm Teleop: Tracking Interrupted!')
            self.__leap_listener.unregister()
            self.__leap_listener = None
        return self.is_tracking    
    
    def toggle_teleop_mode(self, update=True):
        
        if update:
            self.teleop_mode = Teleop_Mode.base_pos \
                if self.teleop_mode == Teleop_Mode.marker_pos \
                else Teleop_Mode.marker_pos
        
        if self.teleop_mode == Teleop_Mode.base_pos:
            rospy.loginfo('Arm Teleop Mode: Absolute!')
        else:
            rospy.loginfo('Arm Teleop Mode: Relative to Marker!')
            
        return str(self.teleop_mode)
            
            
    @property         
    def is_tracking(self):
        return self.__leap_listener is not None              

    def filter_pose(self, pos, quat):
        self.position_buffer.append(np.asarray(pos))
        self.orientation_buffer.append(np.asarray(quat))

        new_pos = np.asarray(self.position_buffer).mean(axis=0)
        quats = np.asarray(self.orientation_buffer)
        # print(quats.dot(quats.T).shape)
        val, vec = np.linalg.eig(quats.T.dot(quats))
        # new_quat = quats.mean(axis=0)#vec[np.argmin(val)]
        new_quat = vec[np.argmax(val)]
        return new_pos, new_quat

    def go_to_pose(self, frame):

        stamped_target = PoseStamped()
        stamped_target.header.frame_id = 'world'
        stamped_target.pose = toMsg(frame)
        self.last_sent_target = stamped_target
        self.send_target(absolute=False, query=stamped_target)
             
    def OnLeapMessage(self, human_msg):
        
        if self.right_hand_mode:
            leap_hand = human_msg.right_hand
        else:
            leap_hand = human_msg.left_hand
            
        if leap_hand.is_present:
            leap2wrist = self.tf_buffer.lookup_transform(
                'world', 'right_leap_hand_wrist', rospy.Time(0), rospy.Duration(0.1))
            # leap2wrist
            wrist_f = fromTf(transform_to_vec(leap2wrist))
            
            if self.previous_tf is not None:
                
                rot_dist = 1 - np.asarray(list(self.target.M.GetQuaternion())).dot(list(self.target.M.GetQuaternion()))
                pos_dist = np.linalg.norm(np.asarray(list(self.previous_tf.p)) - np.asarray(list(wrist_f.p)))
                                          
                if (rot_dist > 1e-3) or (pos_dist > 1e-5):
                    stamped_target = PoseStamped()
                    stamped_target.header.frame_id = 'world'
                    if self.teleop_mode == Teleop_Mode.marker_pos:
                        
                        self.target.p = (wrist_f.p - self.first_tf.p) * 0.5
                        self.target.M = Rotation.Quaternion(*[-0.707, -0.000, 0.707, -0.000]) * wrist_f.M
                        self.go_to_pose(self.target)
                        # stamped_target.pose = toMsg(self.target)
                        # self.send_target(absolute=False, query=stamped_target)
                    else:
                        
                        self.target.p = (wrist_f.p - self.first_tf.p + self.current_pose.p) * 0.5
                        self.target.M = wrist_f.M
                        
                        rospy.loginfo(rospy.get_name() + ': ' + str(np.array(list(self.target.p)).round(3)))
                        self.target_marker.pose = toMsg(self.target)
                        self.marker_pub.publish(self.target_marker)
                        self.go_to_pose(self.target)
                        # stamped_target.pose = toMsg(self.target)
                        # self.posegoal_pub.publish(stamped_target)
                        # self.last_sent_target = stamped_target
                        self.send_target(absolute=True, query=stamped_target)
            else:
                self.first_tf = wrist_f
            self.previous_tf = wrist_f
        else:
            # self.first_tf = 
            self.previous_tf = None
            if self.last_sent_target is not None and self.is_tracking and self.teleop_mode == Teleop_Mode.marker_pos:
                self.send_target(absolute=True, query=self.last_sent_target)