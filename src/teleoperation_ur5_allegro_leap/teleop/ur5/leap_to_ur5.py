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
import os
import tf
import math

import rospy
import roslaunch
import rospkg

from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_leap_teleop.config import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame, config_file_name

from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from leap_motion.msg import leapros, Human, Hand, Finger, Bone
# from abc import ABCMeta, abstractmethod
from utils import pose2str





# Class responsible of defining the reading of the leapmotion data topic and providing it to the main app
class Leap_Teleop_RelaxedIK(object):

    def __init__(self, leap_topic='/leap_motion/leap_device', lefthand_mode=False, expirancy_in_secs=.3, consumption_rate=60):
        rospy.loginfo('Initialization....')
        # initialise class parameters
        self.idx = 0
        self.leap_topic = leap_topic
        self.expirancy_in_secs = expirancy_in_secs
        self.consumption_rate = consumption_rate
        self.lefthand_mode = lefthand_mode
        self.__set_nodeparams()
        self.latest_config = {
            'joint_state': starting_config, 'ee_pose': Pose()}
        # self.latest_config['js'] = starting_config
        # self.latest_config['ee_pose'] = Pose()
        print(pose2str(self.latest_config['ee_pose']))
        ####################################################################################################################
        # Show the Values that are actually used
        rospy.loginfo('Node Name: ' + rospy.get_name())
        rospy.loginfo(['Left' if left else 'Right' for left in [
                      self.lefthand_mode]][0] + ' Hand')
        rospy.loginfo('Leap Topic: ' + self.leap_topic)
        rospy.loginfo('Leap msg expirancy: ' + '%.2f s' %
                      (self.expirancy_in_secs))
        rospy.loginfo('Consumption Rate: ' + '%.2f Hz' %
                      (self.consumption_rate))

        ####################################################################################################################
        # initialise subscribers and publishers
        self.tf_listener = tf.TransformListener()
        self.relaxedIK = RelaxedIK.init_from_config(config_file_name)
        self.js_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        self.ee_pose_goals_pub = rospy.Publisher(
            '/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=3)
        self.max_memory = rospy.Duration(self.expirancy_in_secs)
        self.latest_human_data = []  # queue holding the latest samples in
        self.leap_subscriber = None
        rospy.on_shutdown(self.shutdown)
        rate = rospy.Rate(self.consumption_rate)

        rospy.loginfo('Initialization Complete!')

        ####################################################################################################################
        # Begin listening to leap & ROS loop
        rate = rospy.Rate(self.consumption_rate)
        self.listen_leap()
        rospy.loginfo('Leap Listening started!')
        while not rospy.is_shutdown():
            # static_transform()
            self.consume_data()
            rate.sleep()

    def __set_nodeparams(self):
        rospy.init_node('leap_teleop_relaxed_ik',
                        anonymous=False, log_level=rospy.INFO)
        self.leap_topic = rospy.get_param(
            '~leap_topic', default=self.leap_topic)
        self.expirancy_in_secs = rospy.get_param(
            '~max_msg_age_in_sec', default=self.expirancy_in_secs)
        self.consumption_rate = rospy.get_param(
            '~consumption_rate', default=self.consumption_rate)
        self.lefthand_mode = rospy.get_param(
            '~lefthand_mode', default=self.lefthand_mode)
        self.debug_mode = rospy.get_param('~debug_mode', default=False)
        if self.debug_mode:
            self.mark_pub = rospy.Publisher(
                'relaxed_solver_debug', MarkerArray, queue_size=1)
        else:
            self.mark_pub = None

    @property
    def finger_names(self):
        return ["Thumb", "Index", "Middle", "Ring", "Pinky"]

    # This method controls the way the message is used to extract information
    def __OnLeapMessageReceived(self, leap_msg):

        # check the time of the call
        now = leap_msg.header.stamp
        self.latest_human_data.append(leap_msg)
        # delete all the old elements (they are in order so as soon we find a good one all the rest will be)
        for human in self.latest_human_data:
            if now - human.header.stamp < self.max_memory:
                break  # if it is not too old stop throwing away messages
            self.latest_human_data.pop(0)

    def __get_hand_pos_quat(self, leap_hand):
        "Method that extracts all the needed hand info to send to the solver. It hides the frame transformation as well"
        palm_position = leap_hand.palm_center
        hand_xyz = [
            palm_position.x,
            palm_position.y,
            palm_position.z]

        hand_rpy = [
            leap_hand.roll,
            leap_hand.pitch,
            leap_hand.yaw]

        hand_quat = tf.transformations.quaternion_from_euler(
            -hand_rpy[0],
            hand_rpy[1],
            -hand_rpy[2])

        hand_pose = PoseStamped()
        hand_pose.header.frame_id = "leap_hands"
        hand_pose.header.stamp = rospy.Time(0)
        hand_pose.pose.position.x = hand_xyz[2]
        hand_pose.pose.position.y = hand_xyz[1]
        hand_pose.pose.position.z = -hand_xyz[0]

        hand_pose.pose.orientation.x = hand_quat[0]
        hand_pose.pose.orientation.y = hand_quat[1]
        hand_pose.pose.orientation.z = hand_quat[2]
        hand_pose.pose.orientation.w = hand_quat[3]
        hand_pose = self.tf_listener.transformPose('common_world', hand_pose)
        return hand_pose.pose

    def __publish_pose_marker(self, Markers):
        M = MarkerArray()
        for m in Markers:
            M.markers.append(m)
        self.mark_pub.publish(M)

    def __generate_pose_marker(self, marker_pose, ref_frame='world', name_space='leap_pose', RGBA=[1., 0., 0., .7]):
        m = Marker(type=Marker.ARROW, ns=name_space, action=Marker.ADD)
        m.header.frame_id = ref_frame
        m.pose = marker_pose
        m.scale.x = .2
        m.scale.y = .08
        m.scale.z = .03
        m.color.r = RGBA[0]
        m.color.g = RGBA[1]
        m.color.b = RGBA[2]
        m.color.a = RGBA[3]
        return m

    def __ee_marker(self, ref_frame='common_world', name_space='EE', RGBA=[0., 1., 0., .7]):
        m = Marker(type=Marker.SPHERE, ns=name_space, action=Marker.ADD)
        m.header.frame_id = ref_frame
        m.scale.x = m.scale.y = m.scale.z = .2
        m.color.r = RGBA[0]
        m.color.g = RGBA[1]
        m.color.b = RGBA[2]
        m.color.a = RGBA[3]
        ee_current = self.relaxedIK.vars.ee_positions
        m.pose.position.x = ee_current[0][0]
        m.pose.position.y = ee_current[0][1]
        m.pose.position.z = ee_current[0][2]
        return m

    def __process_hand(self, leap_hand):
        js = None
        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.header.seq = self.idx
        if leap_hand.is_present:
            num_ee = self.relaxedIK.vars.robot.numChains

            hand_pose = self.__get_hand_pos_quat(leap_hand)
            self.latest_config['ee_pose']

            obj_pose = deepcopy(hand_pose)
            obj_pose.position.x += self.latest_config['ee_pose'].position.x
            obj_pose.position.y += self.latest_config['ee_pose'].position.y
            obj_pose.position.z += self.latest_config['ee_pose'].position.z

            hand_xyz = [obj_pose.position.x,
                        obj_pose.position.y, obj_pose.position.z]
            hand_quat = [obj_pose.orientation.x, obj_pose.orientation.y,
                         obj_pose.orientation.z, obj_pose.orientation.w]
            xopt = self.relaxedIK.solve([hand_xyz], [hand_quat])
            js = joint_state_define(xopt)

            now = rospy.Time.now()
            js.header.stamp.secs = now.secs
            js.header.stamp.nsecs = now.nsecs

            ee_pose_goals.ee_poses.append(hand_pose)
            self.latest_config['joint_state'] = xopt
            self.latest_config['ee_pose'] = hand_pose

        else:

            # self.__get_hand_pos_quat(leap_hand)
            hand_pose = self.latest_config['ee_pose']
            # hand_pose.position.x = hand_pose.position.y = hand_pose.position.z = 0
            # hand_pose.orientation.x = hand_pose.orientation.y = hand_pose.orientation.z = 0
            # hand_pose.orientation.w = 1
            # hand_xyz = [hand_pose.position.x, hand_pose.position.y, hand_pose.position.z]
            # hand_quat = [hand_pose.orientation.x, hand_pose.orientation.y, hand_pose.orientation.z, hand_pose.orientation.w]
            # xopt = self.relaxedIK.solve([hand_xyz], [hand_quat])
            js = joint_state_define(self.latest_config['joint_state'])
            # if self.idx == 0:
            #     js = joint_state_define(starting_config)
            # else:
            #     js = joint_state_define(xopt)
            now = rospy.Time.now()
            js.header.stamp.secs = now.secs
            js.header.stamp.nsecs = now.nsecs
            ee_pose_goals.ee_poses.append(self.latest_config['ee_pose'])

        return js, ee_pose_goals

    def listen_leap(self):
        rospy.loginfo('>>> Subscriber Started!')
        self.leap_subscriber = rospy.Subscriber(
            self.leap_topic, Human, self.__OnLeapMessageReceived, queue_size=1)

    def pause_leap(self):
        rospy.loginfo('>>> Subscriber Paused!')
        self.leap_subscriber.unregister()
        self.leap_subscriber = None

    def shutdown(self):
        rospy.loginfo('Stop Listening leap data....')
        self.pause_leap()
        rospy.loginfo('Closing node....')
        rospy.sleep(.5)

    def consume_data(self):
        print('banana')
        if len(self.latest_human_data) > 0:
            # print('here')
            leap_data = self.latest_human_data.pop(0)
            if self.lefthand_mode:
                leap_hand = leap_data.left_hand
            else:
                leap_hand = leap_data.right_hand
            js, ee_pose_goals = self.__process_hand(leap_hand)

            if js is not None:
                self.js_pub.publish(js)
                self.ee_pose_goals_pub.publish(ee_pose_goals)
                if self.debug_mode:
                    rospy.logdebug(self.idx, '\n', pose2str(
                        ee_pose_goals.ee_poses[-1]))
                    M = [
                        self.__ee_marker(),
                        self.__generate_pose_marker(
                            ee_pose_goals.ee_poses[-1]),
                        self.__generate_pose_marker(ee_pose_goals.ee_poses[-1], name_space='leap_frame', ref_frame='leap_hands', RGBA=[0., 0., 1., .7])]
                    self.__publish_pose_marker(M)
                self.idx += 1


if __name__ == '__main__':

    Leap_listener = Leap_Teleop_RelaxedIK()
