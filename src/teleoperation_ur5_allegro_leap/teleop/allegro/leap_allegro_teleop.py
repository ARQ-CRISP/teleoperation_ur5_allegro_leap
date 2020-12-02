#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 15/11/19
'''
from __future__ import print_function, division
from collections import OrderedDict
import tf
# import tf2_geometry_msgs
# import tf2_ros
from scipy.spatial.distance import pdist,cdist
# from copy import deepcopy
# import sys

import pprint

import rospy
import actionlib
import numpy as np
from scipy.optimize import minimize
# from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from leap_motion.msg import leapros, Human, Hand, Finger, Bone
from allegro_hand_kdl.srv import PoseRequest, PoseRequestRequest, PoseRequestResponse
# from allegro_hand_kdl.msg import PoseControlActionGoal, PoseControlActionFeedback, PoseControlActionResult, PoseControlAction
from allegro_hand_kdl.msg import PoseControlAction, PoseControlGoal, PoseControlResult, PoseControlFeedback
from tf.transformations import quaternion_multiply
from moveit_commander.conversions import pose_to_list
# from moveit_commander.conversions import pose_to_list, list_to_pose, list_to_pose_stamped, transform_to_list, list_to_pose_stamped, list_to_transform
# from scipy import optimize
# from kinematic_retargeting import kinematic_retargeting_pose_targets

from visualization_msgs.msg import Marker, MarkerArray
from utils import pose2str, generate_tf, generate_pose_marker, list2ROSPose, ROSPose2list, STransform2SPose
from allegro_state import Allegro_Finger_State
from allegro_utils import finger_allegro_idx, allegro_fingers, common_allegro_poses#, compute_target_state_relax, allegro_finger2linklist
from leap_state import Leap_Hand_TF_Tracker


class Leap_Teleop_Allegro():

    finger_allegro_idx = finger_allegro_idx
    # {'Index': 0, 'Middle': 1, 'Ring': 2, 'Pinky': None, 'Thumb': 3}

    pose_param = "/allegro_hand_kdl/poses/cartesian_poses/"
    pose_index = 2
    pose_cartesian_r_param = "/allegroHand_right_0/gains/cartesian/pose/r"
    control_base_frame = 'hand_root'

    class Control_Type():
        velocity = 0
        position = 1
        position_velocity = 2

    def __init__(self, tf_buffer, leap_topic='/leap_motion/leap_filtered', lefthand_mode=False, scale=1.0, control_type=Control_Type.position_velocity):
        self.__finger2linklist = None
        self.latest_teleop_state = None
        self.control_type = control_type
        self.tf_buffer = tf_buffer
        self.marker_pub = rospy.Publisher(
            'allegro_teleop_debug', MarkerArray, queue_size=5)
        self.__calibration_mode = False
        if type(scale) is list and len(scale) == 4:
            self.scale = OrderedDict([(name, s) for name, s in zip(
                ['Thumb', 'Index', 'Middle', 'Ring'], scale)])
        elif type(scale) is float:
            self.scale = OrderedDict([(name, scale)
                               for name in ['Thumb', 'Index', 'Middle', 'Ring']])
        else:
            print('Error: scale should be a float or an array of dim 4!')
        rospy.loginfo(rospy.get_name() + ': Initialization....')
        self.lefthand_mode = lefthand_mode
        self.leap_topic = leap_topic
        rospy.sleep(2.0)
        self.init_allegro_tips = OrderedDict()
        self.allegro_state = OrderedDict()
        for finger_name, value in finger_allegro_idx.items():
            if value is not None:
                self.allegro_state[finger_name] = Allegro_Finger_State(
                    finger_name, self.tf_buffer)
        self.leap_hand_tracker = Leap_Hand_TF_Tracker(
            self.tf_buffer,  # buffer recycling is good
            self.control_base_frame,  # hand_root
            self.lefthand_mode,  # same mode as teleop
            tracked_fingers=allegro_fingers, #tracks the fingers specified in allegro_utils
            tracked_sections=[0, 1, 2, 3])  # Tracking only base and Fingertip

        self.__leap_listener = rospy.Subscriber(
            self.leap_topic, Human, self._OnLeapReceived, queue_size=1)

        self.__pose_action_client = actionlib.ActionClient('pose_control_action', PoseControlAction)
        self.__pose_action_client.wait_for_server()

    @property
    def leap_hand_str(self):
        return "left_" if self.lefthand_mode else "right_"

    @property
    def is_calibrating(self):
        return self.__calibration_mode

    @property
    def is_tracking(self):
        return self.__leap_listener is not None

    def toggle_calibration_mode(self):
        rospy.loginfo('Allegro Teleop: switching calibration mode!')
        self.__calibration_mode = not self.__calibration_mode

    def toggle_tracking(self):
        if self.__leap_listener is None:
            rospy.loginfo('Allegro Teleop: Resuming Tracking!')
            self.__leap_listener = rospy.Subscriber(
                self.leap_topic, Human, self._OnLeapReceived, queue_size=1)
        else:
            rospy.loginfo('Allegro Teleop: Tracking Interrupted!')
            self.__leap_listener.unregister()
            self.__leap_listener = None

    def on_shutdown(self):
        rospy.loginfo(rospy.get_name() + ': Stop Listening leap data....')
        self.__leap_listener = None
        rospy.loginfo(rospy.get_name() + ': Closing node....')
        rospy.sleep(.5)

    def gen_finger_marker(self, pose, markers, finger_name):
        leap_hand_str = "left_" if self.lefthand_mode else "right_"
        idx = finger_allegro_idx[finger_name]
        m = generate_pose_marker(
            pose,
            ref_frame='hand_root',
            name_space=leap_hand_str + finger_name,
            RGBA=[1. - idx*.25, 1., 0. + idx*.25, .7],
            scale=[0.02, 0.015, 0.01])
        markers.markers.append(m)

    def _OnLeapReceived(self, leap_human):
        # leap_human = Human
        leap_hand = leap_human.left_hand if self.lefthand_mode else leap_human.right_hand
        if leap_hand.is_present:
            markers = MarkerArray()
            try:
                self.leap_hand_tracker.measure_state(leap_hand.header.stamp)
                self.publish_targets(markers, leap_hand.header.stamp)
            except (tf.LookupException, tf.ExtrapolationException) as e:
                print(e)
        else:
            # Reset the state of the teleop otherwhise I am getting peaks in the derivative
            self.leap_hand_tracker.reset_history()

    def get_position_target(self, leap_finger, scale=1.6):

        ee_position = leap_finger.position[-1, :] * scale
        magnitude = np.linalg.norm(ee_position)
        relative_magnitude = magnitude / leap_finger.finger_length()
        target_magnitude = relative_magnitude * \
            self.allegro_state[leap_finger.name].finger_length()
        direction = ee_position / magnitude
        target_pos = direction * target_magnitude
        target_orient = quaternion_multiply(
            leap_finger.orientation[-1, :],
            [0, 0.3826834, 0, 0.9238795])  # 45 deg

        return [target_pos, target_orient]


    def update_position_targets(self, time):

        for finger_name, finger in self.leap_hand_tracker.fingers.items():
            if finger.name is not 'Thumb':
                scale = 1.6
            else:
                scale = 1.0
            target_pose = self.get_position_target(finger, scale)
            self.allegro_state[finger_name].goto(
                target_pose[0],
                target_pose[1],
                time)


    def update_velocity_targets(self, time):
        if self.leap_hand_tracker.history_len > 1:
            for finger_name, finger in self.leap_hand_tracker.fingers.items():
                if finger.velocity is not None:
                    self.allegro_state[finger_name].translate_by(
                        self.scale[finger_name] * finger.velocity[-1, :], time=time)
                    self.allegro_state[finger_name].ee_orientation = finger.orientation[-1, :]

    def update_position_velocity_targets(self, time, position_weight=0.9):

        for finger_name, finger in self.leap_hand_tracker.fingers.items():
            if finger.name is not 'Thumb':
                scale = 1.6
            else:
                scale = 1.1
            target_pose = self.get_position_target(finger, scale)
            if finger.velocity is not None > 1:
                self.allegro_state[finger_name].translate_by(
                    self.scale[finger_name] * finger.velocity[-1, :], time=time)
                target_pose[0] = self.allegro_state[finger_name].ee_position * (1 - position_weight) + target_pose[0] * position_weight
            self.allegro_state[finger_name].goto(target_pose[0], target_pose[1])

    def correct_targets(self, eps=0.01):
        def objfunc(x, hand_dists):
            # hand_dists = pdist(hand_x.reshape(4, 3))
            # print(x.shape)
            # allegro_dists = pdist(x.reshape(4, 3), metric='cosine')
            # f =  np.linalg.norm(allegro_dists - hand_dists) #+ np.linalg.norm(np.linalg.norm(x[-3:]) - hand_dists[-1])
            f = cdist(x[np.newaxis,:], hand_dists[np.newaxis, :], metric='cosine')
            return f
        
        allegro_pos = []
        hand_pos = []
        allegro_finger_sizes = []
        for finger_name, finger in self.leap_hand_tracker.fingers.items():
            allegro_pos.append(self.allegro_state[finger_name].ee_position)
            hand_pos.append(finger.position[-1, :])
            allegro_finger_sizes.append(self.allegro_state[finger_name].finger_length())

        allegro_pos = np.concatenate(allegro_pos)
        hand_pos = np.concatenate(hand_pos)
        res = minimize(
            # lambda x: objfunc(x, pdist(hand_pos.reshape(4, 3), metric='cosine')),
            lambda x: objfunc(x, hand_pos),
            x0=allegro_pos,
            method='SLSQP',
            bounds=[(allegro_pos[i] - eps, allegro_pos[i] + eps) for i in range(12)])

        for i, (finger_name, finger) in enumerate(self.leap_hand_tracker.fingers.items()):
            self.allegro_state[finger_name].ee_position = res.x.reshape(4, 3)[i, :]
        # print(res.x.reshape(4,3))

    def publish_targets(self, markers, time):
        target_state = dict()
        if self.leap_hand_tracker.history_len > 0:
            if self.control_type == self.Control_Type.position:
                self.update_position_targets(time)

            elif self.control_type == self.Control_Type.velocity:
                if self.leap_hand_tracker.history_len > 1:
                    self.update_velocity_targets(time)

            elif self.control_type == self.Control_Type.position_velocity:
                self.update_position_velocity_targets(time, position_weight=0.85)

            # self.correct_targets(eps=0.02)

            posegoal = PoseControlGoal()
            # posegoal.cartesian_pose = [None] * 4
            for finger_name, finger in self.leap_hand_tracker.fingers.items():
                finger_pose = self.allegro_state[finger_name].to_PoseStamped(time).pose
                self.gen_finger_marker(
                    finger_pose, markers, finger_name)
                # posegoal.cartesian_pose[finger_allegro_idx[finger_name]] = finger_pose
                posegoal.cartesian_pose.append(finger_pose)
                # target_state.update(
                #     self.allegro_state[finger_name].to_target_dict(True))
            
            self.__pose_action_client.send_goal(posegoal, feedback_cb=self.on_pose_goal_feedback)

        # print(self.allegro_state)
        if not self.__calibration_mode:
            self.advertise_targets(target_state)
        self.marker_pub.publish(markers)

    def on_pose_goal_feedback(self, feedback):
        """Callback for the feedback message in the PoseControlAction

        Args:
            feedback (PoseControlActionFeedback): Feedback from the PoseControlAction server
            desired (list of arrays, optional): The desired target of the pose action.
        """
        # feedback = PoseControlFeedback()

        if feedback.status.SUCCEEDED:
            for pose, finger_name in zip(feedback.cartesian_pose, allegro_fingers):
                array_pos = np.array(pose_to_list(pose))[:3]
                desidered_pos = np.array(self.allegro_state[finger_name].ee_pose[:3])
                distance = np.sqrt(((array_pos - desidered_pos)**2).sum())
                if distance > 1e-3:
                    rospy.logwarn('The distance of fo the reached point and the desidered position is above Threshold')

    def advertise_targets(self, target_state):
        rospy.logdebug("-"*50)
        rospy.logdebug(rospy.get_name() + ": advertising targets....")
        rospy.set_param(self.pose_param +
                        'p{}/'.format(self.pose_index) + "name", "leap_pose")
        rospy.set_param(self.pose_param +
                        'p{}/'.format(self.pose_index) + "state", target_state)
        self.__last_advertised_terget = target_state
        # print(rospy.get_param(pose_param + "state", "leap_pose"))
        # self.goto_pose_by_name('leap_pose')



    def goto_pose_by_name(self, name='relax'):
        # rospy.wait_for_service('/desired_cartesian_pose')
        # test = rospy.ServiceProxy('/desired_cartesian_pose', PoseRequest)
        if name in common_allegro_poses.keys():
            
            self.__pose_action_client.send_goal(PoseControlGoal(cartesian_pose=common_allegro_poses[name]))#, feedback_cb=self.on_pose_goal_feedback)
        else:
            rospy.logerr(rospy.get_name() + 'Impossible reaching the requested pose. The name {} is not defined!')
        # try:
        #     # call the pose
        #     request = PoseRequestRequest()
        #     request.pose = name
        #     request.behaviour = ''
        #     request.active_fingers = []
        #     query = test(request)
        #     rospy.logdebug(rospy.get_name() + ": targets sent successfully!")
        # except rospy.ServiceException as exc:
        #     rospy.logerr(
        #         rospy.get_name() + ": Service did not process request: " + str(exc.message))
