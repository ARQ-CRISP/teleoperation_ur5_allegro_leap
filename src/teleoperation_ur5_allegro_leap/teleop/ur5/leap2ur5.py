#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 5/10/19
'''
######################################################################################################

from __future__ import print_function
from copy import deepcopy
import tf
import tf2_geometry_msgs
import tf2_ros
import os
import tf
import math

import rospy
import roslaunch
import rospkg

from relaxed_wrapper import Relaxed_UR5_Controller
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from leap_motion.msg import leapros, Human, Hand, Finger, Bone
# from abc import ABCMeta, abstractmethod
from utils import pose2str, STransform2SPose, generate_pose_marker, list2ROSPose


# Class responsible of defining the reading of the leapmotion data topic and providing it to the main app
class Leap_Teleop_RelaxedIK(object):

    def __init__(self, leap_topic='/leap_motion/leap_device', lefthand_mode=False, expirancy_in_secs=.3, consumption_rate=60):
        self.__finger2linklist = None
        self.current_teleopstate = None
        self.leap_topic = leap_topic
        self.lefthand_mode = lefthand_mode
        self.expirancy_in_secs = expirancy_in_secs
        self.consumption_rate = consumption_rate

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.leap_listener = rospy.Subscriber(
            self.leap_topic, Human, self.__OnLeapMessageReceived, queue_size=10)
        self.marker_pub = rospy.Publisher(
            'allegro_teleop_debug', MarkerArray, queue_size=5)
        self.controller = Relaxed_UR5_Controller()
    @property
    def leap_hand_str(self):
        return "left_" if self.lefthand_mode else "right_"

    @property
    def finger_names(self):
        return ["Thumb", "Index", "Middle", "Ring", "Pinky"]

    def generate_state(self, new_teleop_state, markers):
        try:
            # print(time)
            time = new_teleop_state['time']
            teleop_state['leap2allegro'] = self.lookup_leap2hand_palm_transform(
                time)
            leap2allegro = STransform2SPose(new_teleop_state['leap2allegro'])
            m = generate_pose_marker(
                leap2allegro.pose, ref_frame='hand_root', name_space='palm_link')
            markers.markers.append(m)
            new_teleop_state['palm_pose'] = dict()
            self.gen_palm_state(new_teleop_state, markers)

            # self.publish_goals(teleop_state)
            # print(leap2allegro)
        except (tf.LookupException, tf.ExtrapolationException) as e:
            print(e)

    def gen_palm_state(self, new_teleop_state, markers):
        time = new_teleop_state['time']
        leap2allegro = new_teleop_state['leap2allegro']
        w2wrist = self.lookup_world2leap_hand(time, 'leap_hand_wrist')
        w2center = self.lookup_world2leap_hand(time, 'leap_hand_center')
        new_teleop_state['palm_pose']['leap'] = {
            'wrist': w2wrist,
            'center': w2center}

        #TODO add module to get pose from RelaxedIK
        new_teleop_state['palm_pose']['UR5'] = {
            'wrist': None,
            'center': None}

    def lookup_world2leap_hand(self, time, leap_link='leap_hand_wrist'):

        hand_base = self.leap_hand_str + leap_link
        base = 'world'
        
        self.tf_buffer.can_transform(
            base, hand_base, time, rospy.Duration(0.1))
        wrist_pose = self.tf_buffer.lookup_transform(
            base, hand_base, time, rospy.Duration(0.2))
        return STransform2SPose(wrist_pose)
        

    def lookup_leap2hand_palm_transform(self, time):

        leap2palm = self.tf_buffer.lookup_transform(
            'palm_link', 'leap_hands', time, rospy.Duration(0.1))
        leap2palm.transform.translation = Vector3(0, 0, 0)
        return leap2palm

    def publish_goals(self, new_teleop_state, markers):
        print('publish now')
        ur5_palm_pos = self.controller.EE_position
        self.controller.go_to(ur5_palm_pos[0],[0,0,0,1])
        palmPose = list2ROSPose(ur5_palm_pos[0], [0., 0., 0., 1.])
        m = generate_pose_marker(
                palmPose, ref_frame='common_world', name_space='ur5_control')
        markers.append(m)

        self.marker_pub.publish(markers)

    # This method controls the way the message is used to extract information

    def __OnLeapMessageReceived(self, leap_msg):
        # leap_msg = Human()
        if self.lefthand_mode:
            leap_hand = leap_msg.left_hand
        else:
            leap_hand = leap_msg.right_hand
        # print(leap_hand)
        if leap_hand.is_present:
            # print("hand present")       
            new_teleop_state = dict()
            new_teleop_state['time'] = leap_msg.header.stamp
            markers = MarkerArray()
            try:
                self.generate_state(new_teleop_state, markers)
                self.publish_goals(new_teleop_state, markers)
            except (tf.LookupException, tf.ExtrapolationException) as e:
                print(e)
        else:
            # Reset the state of the teleop otherwhise I am getting peaks in the derivative
            self.current_teleopstate = None

    # def __set_nodeparams(self):
    #     rospy.init_node('leap_teleop_relaxed_ik',
    #                     anonymous=False, log_level=rospy.INFO)
    #     self.leap_topic = rospy.get_param(
    #         '~leap_topic', default=self.leap_topic)
    #     self.expirancy_in_secs = rospy.get_param(
    #         '~max_msg_age_in_sec', default=self.expirancy_in_secs)
    #     self.consumption_rate = rospy.get_param(
    #         '~consumption_rate', default=self.consumption_rate)
    #     self.lefthand_mode = rospy.get_param(
    #         '~lefthand_mode', default=self.lefthand_mode)
    #     self.debug_mode = rospy.get_param('~debug_mode', default=False)
    #     if self.debug_mode:
    #         self.mark_pub = rospy.Publisher(
    #             'relaxed_solver_debug', MarkerArray, queue_size=1)
    #     else:
    #         self.mark_pub = None


if __name__ == '__main__':
    rospy.init_node('UR5_teleop')
    Leap_listener = Leap_Teleop_RelaxedIK(leap_topic='/leap_motion/leap_filtered')
    rospy.spin()
