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
from  scipy.spatial.transform import Rotation as R
from  scipy.spatial.transform import Slerp
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
from teleoperation_ur5_allegro_leap.msg import Control_Type
from teleoperation_ur5_allegro_leap.srv import Toggle_Tracking, Toggle_TrackingResponse
from teleoperation_ur5_allegro_leap.srv import Toggle_Calibration, Toggle_CalibrationResponse
from teleoperation_ur5_allegro_leap.srv import Toggle_Control, Toggle_ControlResponse
from teleoperation_ur5_allegro_leap.srv import GoTo_ByName, GoTo_ByNameResponse
from teleoperation_ur5_allegro_leap.srv import Get_Fingertip_Distance, Get_Fingertip_DistanceResponse
from teleoperation_ur5_allegro_leap.srv import Update_Finger_Measure, Update_Finger_MeasureResponse


from .utils import pose2str, generate_tf, generate_pose_marker, list2ROSPose, ROSPose2list, STransform2SPose
from .allegro_state import Allegro_Finger_State, Allegro_Thumb_State, Allegro_Hand_State
from .allegro_utils import finger_allegro_idx, allegro_fingers, common_allegro_poses#, compute_target_state_relax, allegro_finger2linklist
from .leap_state import Leap_Hand_TF_Tracker

Controllers_Dict = {
    'position': Control_Type.position,
    'velocity': Control_Type.velocity, 
    'position_velocity' : Control_Type.position_velocity, 
    'joint_position': Control_Type.joint_position}

class Leap_Teleop_Allegro():

    finger_allegro_idx = finger_allegro_idx
    # {'Index': 0, 'Middle': 1, 'Ring': 2, 'Pinky': None, 'Thumb': 3}

    toggle_tracking_srv = 'allegro_teleop/toggle_tracking'
    toggle_calibration_srv = 'allegro_teleop/toggle_calibration'
    pose_by_name_srv = 'allegro_teleop/pose_by_name'
    set_controller_srv = 'allegro_teleop/set_controller'
    fingertip_distance_srv = 'allegro_teleop/fingertip_distance'
    fingertip_update_srv = 'allegro_teleop/fingertip_update'
    
    pose_param = "/allegro_hand_kdl/poses/cartesian_poses/"
    pose_index = 2
    pose_cartesian_r_param = "/allegroHand_right_0/gains/cartesian/pose/r"
    control_base_frame = 'right_leap_fingers'

    def __init__(self, tf_buffer, leap_topic='/leap_motion/leap_filtered', lefthand_mode=False, scale=1.0, control_type=Control_Type.position_velocity):
        self.__finger2linklist = None
        self.latest_teleop_state = None
        self.__control_type = control_type
        self.tf_buffer = tf_buffer
        self.marker_pub = rospy.Publisher(
            'allegro_teleop_debug', MarkerArray, queue_size=5)
        self.__calibration_mode = False
        if type(scale) is list and len(scale) == 4:
            self.scale = OrderedDict([(name, s) for name, s in zip(
                allegro_fingers, scale)])
        elif type(scale) is float:
            self.scale = OrderedDict([(name, scale)
                               for name in allegro_fingers])
        else:
            print('Error: scale should be a float or an array of dim 4!')
        rospy.loginfo(rospy.get_name() + ': Initialization....')
        self.lefthand_mode = lefthand_mode
        self.leap_topic = leap_topic
        rospy.sleep(2.0)
        self.init_allegro_tips = OrderedDict()
        self.allegro_state = Allegro_Hand_State(tf_buffer)
        # self.allegro_state = OrderedDict()
        # for finger_name, value in finger_allegro_idx.items():
        #     if value is not None:
        #         if finger_name == 'Thumb':
        #             self.allegro_state[finger_name] = Allegro_Thumb_State(
        #                 finger_name, self.tf_buffer)
        #         else:
        #             self.allegro_state[finger_name] = Allegro_Finger_State(
        #                 finger_name, self.tf_buffer)
                
        self.leap_hand_tracker = Leap_Hand_TF_Tracker(
            self.tf_buffer,  # buffer recycling is good
            self.control_base_frame,  # hand_root
            self.lefthand_mode,  # same mode as teleop
            tracked_fingers=allegro_fingers, #tracks the fingers specified in allegro_utils
            tracked_sections=[0, 1, 2, 3, 4])  # Tracking only base and Fingertip

        
        self.__leap_listener = rospy.Subscriber(
            self.leap_topic, Human, self._OnLeapReceived, queue_size=1)

        self.__pose_action_client = actionlib.ActionClient('pose_control_action', PoseControlAction)
        self.__pose_action_client.wait_for_server()
        
        self.__tracking_toggler = rospy.Service(self.toggle_tracking_srv, Toggle_Tracking,
                                                lambda update: Toggle_TrackingResponse( 
                                                    self.toggle_tracking() if update.update else self.is_tracking))
        self.__calibration_toggler = rospy.Service(self.toggle_calibration_srv, Toggle_Calibration,
                                                lambda update: Toggle_CalibrationResponse(
                                                    self.toggle_calibration_mode() if update.update else self.is_calibrating))
        self.__pose_by_name = rospy.Service(self.pose_by_name_srv, GoTo_ByName,
                                                lambda name: GoTo_ByNameResponse(self.goto_pose_by_name(name.posename)))
        self.__set_controller = rospy.Service(self.set_controller_srv, Toggle_Control,
                                                lambda controller: Toggle_ControlResponse(self.set_control_type(controller.type_requested)))
        self.__get_distance = rospy.Service(self.fingertip_distance_srv, Get_Fingertip_Distance,
                                                lambda data: Get_Fingertip_DistanceResponse(
                                                    self.get_fingertip_distance(data.finger1, data.finger2)))
        self.__update_measure = rospy.Service(self.fingertip_update_srv, Update_Finger_Measure,
                                                lambda data: Update_Finger_MeasureResponse(
                                                    self.update_finger_measure(data.finger_name)))

    @property
    def leap_hand_str(self):
        return "left_" if self.lefthand_mode else "right_"

    @property
    def is_calibrating(self):
        return self.__calibration_mode

    @property
    def is_tracking(self):
        return self.__leap_listener is not None

    def set_control_type(self, control_type):
        if int(control_type) in Controllers_Dict.values():
            self.__control_type = control_type
        control_name = [key for key, value in Controllers_Dict.items() if value==self.__control_type][0]
        rospy.loginfo('Control type updated to {}'.format(control_name))
        return self.__control_type

    def toggle_calibration_mode(self):
        rospy.loginfo('Allegro Teleop: switching calibration mode!')
        self.__calibration_mode = not self.__calibration_mode
        return self.__calibration_mode

    def toggle_tracking(self):
        if self.__leap_listener is None:
            rospy.loginfo('Allegro Teleop: Resuming Tracking!')
            self.__leap_listener = rospy.Subscriber(
                self.leap_topic, Human, self._OnLeapReceived, queue_size=1)
        else:
            rospy.loginfo('Allegro Teleop: Tracking Interrupted!')
            self.__leap_listener.unregister()
            self.__leap_listener = None
        return self.is_tracking

    def get_fingertip_distance(self, Finger1, Finger2):
        fing1 = self.leap_hand_tracker.fingers[Finger1].position[-1, :]
        fing2 = self.leap_hand_tracker.fingers[Finger2].position[-1, :]
        return np.linalg.norm(fing1 - fing2)
    
    def update_finger_measure(self, finger_name):
        if self.__control_type in [Control_Type.velocity, Control_Type.position_velocity] and finger_name in self.allegro_state:
            self.allegro_state[finger_name].update_measure()
            return True
        return False
        
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
                for finger_name, value in finger_allegro_idx.items():
                    if value is not None:
                        self.allegro_state[finger_name].update_measure(leap_hand.header.stamp)
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
            # [ 0, 0.5, 0, 0.8660254 ])
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
        def sigmoid(x, scale=1.): return 1/(1 + np.exp(-scale*x))
        index = self.leap_hand_tracker.fingers['Index'].position[-1, :]
        middle = self.leap_hand_tracker.fingers['Middle'].position[-1, :]
        ring = self.leap_hand_tracker.fingers['Ring'].position[-1, :]
        thumb = self.leap_hand_tracker.fingers['Thumb'].position[-1, :]
        center = np.mean([0.2/3*index, 0.2/3*middle, 0.2/3*ring, 0.8*thumb], axis=0)
        dist2thumb = np.asarray([np.linalg.norm(index-thumb), np.linalg.norm(middle-thumb), np.linalg.norm(ring-thumb)])
        dist2thumb /= dist2thumb.max()
        dist2thumb = np.asarray([1.] * 3)
        center_primary = np.mean(np.asarray([index, middle, ring]) * (1 - dist2thumb), axis=0)
        # print('center', center) 
        f = 0.035
        c = 0.05
        for finger_name, finger in self.leap_hand_tracker.fingers.items():
            if finger.name is 'Thumb':
                scale = 1.
            else:
                scale = 1.
            target_pose = self.get_position_target(finger, scale)
            if finger_name == 'Index':
                diff2middle = (index - middle)[[1,2]]
                diff2thumb = (center - index)[[1,2]]
                act  = (1 - sigmoid(np.linalg.norm(diff2middle) - 0.02, 0.9)) * f
                actc = (1 - sigmoid(np.linalg.norm(diff2thumb) - 0.03, 0.7)) * c
                target_pose[0][[1,2]] += diff2middle / np.linalg.norm(diff2middle) * act
                target_pose[0][[1,2]] += diff2thumb  / np.linalg.norm(diff2thumb)  * actc
            elif finger_name == 'Ring': 
                diff2middle = (ring - middle)[[1,2]]
                diff2thumb  =  (center - ring)[[1,2]]
                act  = (1 - sigmoid(np.linalg.norm(diff2middle) - 0.03, 0.6)) * f
                actc = (1 - sigmoid(np.linalg.norm(diff2thumb) - 0.05, 0.7)) * c
                target_pose[0][[1,2]] += diff2middle / np.linalg.norm(diff2middle) * act
                target_pose[0][[1,2]] += diff2thumb / np.linalg.norm(diff2thumb)   * actc
            elif finger_name == 'Thumb':
                rot0 = R.from_quat([
                    [-0.260, 0.205, 0.538, 0.775], # quaternion on lifted thumb (all left)
                    [-0.091, -0.185, -0.012, 0.978]]) #quaternion on low thumb (max_right)
                # thumb_left_up = [-0.01775792  0.08817481  0.0615432 ]
                # thumb_all_left = [0.00969561 0.10123025 0.05504629]
                # thumb_all_down = [0.06546346 0.03441163 0.07661921]
                # thumb_all_right = [ 0.05141937 -0.0124956   0.06841916]
                z = (thumb[1] + 0.013) / (0.10123025 + 0.013)
                # print('thumb pos {} - '.format(thumb), z, sigmoid((1-z - 0.5) * 100))
                # if 0. < z < 1.:
                target_pose[1] = Slerp(np.asarray([0, 1]), rot0)([sigmoid((1 - z - 0.5) * 100)]).as_quat()[0]
                diff_primary = center_primary - thumb
                diff_center = center - thumb
                thumb_move = thumb - self.leap_hand_tracker.fingers['Thumb'].position[-2, :]
                # print('thumb_move: ', thumb_move[1])
                # if np.abs(thumb_move[1]) > 0.01:
                if z > 0.7:
                    target_pose[0][[1]] += thumb_move[1] * 2.5 #increase frontal moving
                    target_pose[0][[2]] -= thumb_move[2] * 2.5 #reduce lateral moving
                act = (1 - sigmoid(np.linalg.norm(diff_primary) - 0.06, 100)) * c
                target_pose[0][[0,1,2]] += (diff_center)[[0,1,2]] / np.linalg.norm(diff_center) * act
            elif finger_name == 'Middle': 
                diff = (center - middle)[[1,2]]
                act = (1 - sigmoid(np.linalg.norm(diff) - 0.03, 0.9)) * 2e-2
                target_pose[0][[1,2]] +=  diff / np.linalg.norm(diff) * act
            
            if finger.velocity is not None > 1:
                self.allegro_state[finger_name].translate_by(
                    self.scale[finger_name] * finger.velocity[-1, :], time=time)
                target_pose[0] = self.allegro_state[finger_name].ee_position * (1 - position_weight) + target_pose[0] * position_weight
            # print(self.allegro_state[finger_name].ee_pose, target_pose[0])
            self.allegro_state[finger_name].goto(self.allegro_state[finger_name].base_position+target_pose[0], target_pose[1])
    
    def update_joint_position_targets(self):
        pass
    
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

    def update_joint_targets(self, time):
        pass
    
    def publish_targets(self, markers, time):
        target_state = dict()
        posegoal = PoseControlGoal()
        if self.leap_hand_tracker.history_len > 0:
            if self.__control_type == Control_Type.position:
                self.update_position_targets(time)

            elif self.__control_type == Control_Type.velocity:
                if self.leap_hand_tracker.history_len > 1:
                    self.update_velocity_targets(time)

            elif self.__control_type == Control_Type.position_velocity:
                self.update_position_velocity_targets(time, position_weight=0.9)
                
            elif self.__control_type == Control_Type.joint_position:
                self.update_joint_targets(time)

            # self.correct_targets(eps=0.02)

            # posegoal.cartesian_pose = [None] * 4
            for finger_name, finger in self.leap_hand_tracker.fingers.items():
                if self.__control_type not in []:
                    finger_pose = self.allegro_state[finger_name].to_PoseStamped(time).pose
                    self.gen_finger_marker(
                        finger_pose, markers, finger_name)
                    posegoal.cartesian_pose.append(finger_pose)
                else:
                    #TODO edit for joint control
                    finger_pose = self.allegro_state[finger_name].to_PoseStamped(time).pose
                    posegoal.joint_pose

        # print(self.allegro_state)
        if not self.__calibration_mode:
            self.__pose_action_client.send_goal(posegoal, feedback_cb=self.on_pose_goal_feedback)
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



    def goto_pose_by_name(self, name='relax'):
        if name in common_allegro_poses.keys():
            
            self.__pose_action_client.send_goal(PoseControlGoal(cartesian_pose=common_allegro_poses[name]))#, feedback_cb=self.on_pose_goal_feedback)
            return True
        else:
            rospy.logerr(rospy.get_name() + 'Impossible reaching the requested pose. The name {} is not defined!')
            return False
        
