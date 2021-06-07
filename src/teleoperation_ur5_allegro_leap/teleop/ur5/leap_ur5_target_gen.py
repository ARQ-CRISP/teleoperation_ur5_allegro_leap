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
from PyKDL import Frame, Rotation

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from leap_motion.msg import Human
from tf_conversions import fromTf, toMsg

from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix

def transform_to_vec(T):
    t = T.transform.translation
    r = T.transform.rotation
    return [[t.x, t.y, t.z], [r.x, r.y, r.z, r.w]]

class Leap_Teleop_UR5():
    
    leap_motion_topic = '/leap_motion/leap_device'
    marker_topic = ur5_teleop_prefix + 'target_marker'
    pose_goal_topic = ur5_teleop_prefix + 'ur5_pose_targets'
    
    def __init__(self, right_hand=True):
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.previous_tf = None
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
        self.leap_subscriber = rospy.Subscriber(self.leap_motion_topic, Human, self.OnLeapMessage, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1)
        self.posegoal_pub = rospy.Publisher(self.pose_goal_topic, PoseStamped, queue_size=1)
             
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
                 
                rospy.loginfo(rospy.get_name() + ': ' + str(np.array(list(self.current_pose.p)).round(3)))
                self.target_marker.pose = toMsg(self.current_pose)
            
                self.marker_pub.publish(self.target_marker )
                stamped_target = PoseStamped()
                stamped_target.header.frame_id = 'world'
                stamped_target.pose = toMsg(self.current_pose)
                self.posegoal_pub.publish(stamped_target)
            
            self.previous_tf = wrist_f