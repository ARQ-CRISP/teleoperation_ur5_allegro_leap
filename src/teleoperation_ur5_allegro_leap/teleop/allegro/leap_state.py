#! /usr/bin/env python
from __future__ import division, print_function

import numpy as np
import rospy
import tf2_ros
from moveit_commander.conversions import (list_to_pose_stamped,
                                          transform_to_list)

from collections import OrderedDict, deque
from allegro_utils import allegro_fingers


class Leap_Hand_TF_Tracker():
    """Leap_Hand_TF_Tracker

    This Class manages the lifecycle of the leap finger tracking for all the fingers
    measure_state()     -- gets a new_measurement for all the fingers from the current TF
    reset_history()     -- resets the state of the tracking
    to_pose()           -- transforms the class to target_poses
    history_len         -- gets the current length of historical values kept in the class 
    last_measure        -- get the last measurement
    position            -- gets the last position of the tracked fingers
    orientation         -- gets the last orientation of the tracked fingers
    velocity            -- gets the last displacement of the tracked fingers
    velocity_normalised -- gets the displacement divided by the time duration in which they were computed
    pose                -- (position, orientation)
    """

    FINGER_ORDER = allegro_fingers

    def __init__(self, tf_buffer, base_frame='hand_root', left_hand_mode=False, tracked_fingers=None, tracked_sections=[4], history_len=3):

        self.base_frame = base_frame
        self.tf_buffer = tf_buffer
        # self.last_measurement_time = time
        self.tracked_fingers = tracked_fingers if tracked_fingers is not None else self.FINGER_ORDER
        self.fingers = OrderedDict([ #setting the fingers as the list of tracked fingers (Ordered dict, allows to get the items in the right order)
             (finger, Leap_Thumb_TF_Tracker(finger, tf_buffer, base_frame, left_hand_mode, tracked_sections, history_len)) if finger == 'Thumb' \
                else (finger, Leap_Finger_TF_Tracker(finger, tf_buffer, base_frame, left_hand_mode, tracked_sections, history_len)) \
                for finger in self.tracked_fingers])
        self.left_hand_mode = left_hand_mode
        self.max_hist = history_len

        # if measure_on_init:
        # self.measure_state(time)

    def measure_state(self, time):
        for finger_name, finger in self.fingers.items():
            finger.measure_state(time)

    def reset_history(self):
        for finger_name, finger in self.fingers.items():
            finger.reset_history()

    def to_pose(self, section_selection=None):
        fingers = []
        for finger_name, finger in self.fingers.items():
            finger_poses = finger.to_pose()
            if section_selection is not None:
                fingers.append(finger_poses)
            else:
                fingers.append([finger_poses[i] for i in section_selection])

        return fingers

    def __str__(self):
        hand_str = 'LEFT' if self.left_hand_mode else 'RIGHT'
        if self.last_measure is not None:
            message = [
                '--> {} HAND at time {:.3f}'.format(hand_str, self.last_measure.to_sec())]
            for name, finger in self.fingers.items():
                message.append(finger.__str__())
            message.append('\n-->\n')
            return ''.join(message)
        else:
            return 'Empty-Hand'

    @property
    def history_len(self):
        return min([len(finger.measurement_times) for name, finger in self.fingers.items()])

    @property
    def last_measure(self):
        return max([finger.last_measure for name, finger in self.fingers.items()])

    @property
    def position(self):
        if (self.history_len) > 0:
            return OrderedDict([(name, finger.position) for name, finger in self.fingers.items()])
        else:
            return None

    @property
    def orientation(self):
        if (self.history_len) > 0:
            return OrderedDict([(name, finger.orientation) for name, finger in self.fingers.items()])
        else:
            return None

    @property
    def velocity(self):
        if (self.history_len) > 1:
            return OrderedDict([(name, finger.velocity) for name, finger in self.fingers.items()])
        else:
            return None

    @property
    def velocity_normalised(self):
        if (self.history_len) > 1:
            return OrderedDict([(name, finger.velocity) for name, finger in self.fingers.items()])
        else:
            return None

    @property
    def pose(self):
        if (self.history_len) > 0:
            return self.state[-1]
        else:
            return None
        
    @property
    def joint_position(self):
        if (self.history_len) > 0:
            jstates = np.zeros(4*4)
            for i, finger in enumerate(self.tracked_fingers):
                print(finger, self.fingers[finger].joint_position[-1])
                jstates[i*4:i*4+4] = self.fingers[finger].joint_position[-1]
            return jstates
        else:
            return None


class Leap_Finger_TF_Tracker(object):
    _section2str = ['base', '0', '1', '2', '3']
    _leap_hand_str = ["right_", "left_"]

    def __init__(self, name, tf_buffer, base_frame='right_leap_fingers', left_hand_mode=False, tracked_sections=None, history_len=3):

        self.name = name
        self.tf_buffer = tf_buffer
        self.left_hand_mode = left_hand_mode
        self.base_frame = base_frame
        self._max_history = history_len
        self._len = None
        if tracked_sections is None:
            self.tracked_sections = self._section2str
        else:
            self.tracked_sections = [self._section2str[i]
                                     for i in tracked_sections]

        self.measurement_times = deque([], history_len)
        # self.state = []
        self.state = deque([], history_len)
        self.joint_position = deque([], history_len)
        # if measure_on_init:
        # self.measure_state(time)

    def finger_length(self):
        if self._len is None:
            time = rospy.Time().now()
            new_state = np.zeros((4, 7))
            for i, section in enumerate(range(2, 4)):
                finger_joint = self._leap_hand_str[int(self.left_hand_mode)] + \
                    self.name + "_" + self._section2str[section]
                self.tf_buffer.can_transform(
                    self.base_frame, finger_joint, time, rospy.Duration(0.1))
                leap_section_transform = self.tf_buffer.lookup_transform(
                    self.base_frame, finger_joint, time, rospy.Duration(0.2))
                new_state[i, :] = np.array(transform_to_list(
                    leap_section_transform.transform))
            self._len = np.linalg.norm(
                np.diff(new_state[:, :3], axis=0), axis=1)[1:].sum()
        return self._len

    def measure_state(self, time):

        # if len(self.state) == self._max_history:
        #     self.state.pop(0)
        #     self.measurement_times.pop(0)

        new_state = np.zeros((len(self.tracked_sections), 7))
        for i, section in enumerate(self.tracked_sections):
            finger_joint = self._leap_hand_str[int(self.left_hand_mode)] + \
                self.name + "_" + section
            self.tf_buffer.can_transform(
                self.base_frame, finger_joint, time, rospy.Duration(0.1))
            leap_section_transform = self.tf_buffer.lookup_transform(
                self.base_frame, finger_joint, time, rospy.Duration(0.2))
            new_state[i, :] = np.array(transform_to_list(
                leap_section_transform.transform))
        self.joint_position.append(self.jangles(new_state))
        self.state.append(new_state)
        self.measurement_times.append(time)

    def reset_history(self):
        if self.history_len > 0:
            self.state.clear()
            # print(self.name)
            # del self.state[:]
            # self.last_measure = []
            self._len = None

    def to_pose(self):
        poses = []
        for i, section in enumerate(self.tracked_sections):
            new_pose = list_to_pose_stamped(
                self.pose[i, :].tolist(), self.base_frame)
            new_pose.header.stamp = self.last_measure
            poses.append(new_pose)

        return poses

    @property
    def history_len(self):
        return len(self.measurement_times)

    @property
    def last_measure(self):
        if self.history_len > 0:
            return self.measurement_times[-1]
        else:
            return None

    @property
    def position(self):
        if len(self.state) > 0:
            return self.state[-1][:, :3]
        else:
            return None

    @property
    def orientation(self):
        if len(self.state) > 0:
            return self.state[-1][:, 3:]
        else:
            return None
        
    def jangles(self, state):
        # print(self.name, state.shape)
        angles = np.zeros(state.shape[0] - 1)
        n1 = np.eye(3)[0] * state[1, :3]
        n2 = np.eye(3)[2] * state[1, :3]
        v0 = state[2, :3] - state[1, :3]
        proj_v0 = v0 - (v0.dot(np.eye(3)[0]) * np.eye(3)[0])
        proj_v0 /= np.linalg.norm(proj_v0)
        angles[0] = np.arctan2(np.cross(proj_v0, n2).dot(np.eye(3)[0]), np.dot(proj_v0, n2))
        for i in range(state.shape[0] - 2):
            cur = state[i+1, :3] - state[i, :3]
            nex = state[i+2, :3] - state[i+1, :3]
            # if  np.linalg.norm(cur)==0 or np.linalg.norm(nex) ==0 :
                # print(np.linalg.norm(cur), np.linalg.norm(nex))
            angles[i+1] = np.arccos(np.dot(cur, nex) / np.linalg.norm(cur) / np.linalg.norm(nex))
        # print(self.name, np.degrees(angles))
        return angles
            

    @property
    def velocity(self):
        if len(self.state) > 1:
            return self.state[-1][:, :3] - self.state[-2][:, :3]
        else:
            return None

    @property
    def velocity_normalised(self):
        if len(self.state) > 1:
            return (self.state[-1][:, :3] - self.state[-2][:, :3]) / (self.measurement_times[-1] - self.measurement_times[-2]).to_sec()
        else:
            return None

    @property
    def pose(self):
        if len(self.state) > 0:
            return self.state[-1]
        else:
            return None

    def __str__(self):
        message = []
        hand_str = 'left' if self.left_hand_mode else 'right'
        # message.append('-'*50)
        if self.history_len > 0:
            message.append('\n- - Measurement at time {:.3f} of the {} {}'.format(
                self.last_measure.to_sec(), hand_str, self.name))
            for i, section in enumerate(self.tracked_sections):
                message.append('section {} position: {:.3f}, {:.3f}, {:.3f}'.format(section,
                                                                                    *self.position[i, :].tolist()))
                message.append('section {} orientation: {:.3f}, {:.3f}, {:.3f}'.format(section,
                                                                                       *self.orientation[i, :].tolist()))
                if self.history_len > 1:
                    message.append('velocity: {:.3f}, {:.3f}, {:.3f}'.format(
                        *self.velocity.tolist()[-1]))
                    message.append('velocity normalised: {:.3f}, {:.3f}, {:.3f}'.format(
                        *self.velocity_normalised.tolist()[-1]))
            # message.append('\n')
            return '\n'.join(message)
        else:
            return 'Empty-Finger'
        
        
class Leap_Thumb_TF_Tracker(Leap_Finger_TF_Tracker):
    def __init__(self, name, tf_buffer, base_frame='right_leap_fingers', left_hand_mode=False, tracked_sections=None, history_len=3):
        
        super(Leap_Thumb_TF_Tracker, self).__init__(
            name, tf_buffer, base_frame, left_hand_mode, tracked_sections, history_len)
        
    def jangles(self, state):
        # print(self.name, state.shape)
        angles = np.zeros(state.shape[0] - 1)
        n1 = np.eye(3)[1]
        proj_v0 = state[3, :3] - (state[3, :3].dot(np.eye(3)[2]) * np.eye(3)[2])
        proj_v0 /= np.linalg.norm(proj_v0)
        angles[0] = np.arctan2(np.cross(proj_v0, n1).dot(np.eye(3)[2]), np.dot(proj_v0, n1)) #np.arccos(n1.dot(n2))
        
        v1 = state[3, :3] - state[2, :3]
        v1 /= np.linalg.norm(v1)
        proj_v1 = v1 - (v1.dot(proj_v0) * proj_v0)
        angles[1] = np.arctan2(np.linalg.norm(np.cross(np.eye(3)[2], proj_v1)), np.dot(np.eye(3)[2], proj_v1))
        
        # n2 = state[2, :3] - state[2+1, :3]
        # n2 = np.cross(n2, np.asarray([0., 0., 1.]))
        # n2 /= np.linalg.norm(n2)
        # print(proj_v)
        # print(n1, n2) 
        # state[0, :3] = [0., 0., 0.]
        for i in range(1, state.shape[0] - 2):
            cur = state[i, :3] - state[i+1, :3]
            nex = state[i+1, :3] - state[i+2, :3]
            if np.linalg.norm(cur) == 0 or np.linalg.norm(nex) == 0 :
                print(np.linalg.norm(cur), np.linalg.norm(nex))
            angles[i+1] = np.arccos(np.dot(cur, nex) / np.linalg.norm(cur) / np.linalg.norm(nex))
        # print(self.name, np.degrees(angles))
        return angles


if __name__ == "__main__":

    rospy.init_node('leap_tracker_tester')
    buffer = tf2_ros.Buffer(rospy.Duration(50))
    listener = tf2_ros.TransformListener(buffer)
    rospy.sleep(.5)
    init_time = rospy.Time().now()
    # finger_tracker = Leap_Finger_TF_Tracker(
    #     name=Leap_Hand_TF_Tracker.FINGER_ORDER[1], tf_buffer=buffer, time=init_time)

    hand_tracker = Leap_Hand_TF_Tracker(buffer, 'hand_root')
    hand_tracker.measure_state(init_time)
    print(hand_tracker)

    rospy.sleep(.1)
    time1 = rospy.Time().now()
    hand_tracker.measure_state(time1)
    print(hand_tracker)

    rospy.sleep(.1)
    time2 = rospy.Time().now()
    hand_tracker.measure_state(time2)
    print(hand_tracker)

    rospy.sleep(.1)
    time3 = rospy.Time().now()
    hand_tracker.measure_state(time3)
    print(hand_tracker)

    for i in range(10):
        rospy.sleep(.1)
        new_time = rospy.Time().now()
        hand_tracker.measure_state(new_time)
        print('measurement {:d}'.format(i))
        print('tracked_history: {:d}'.format(hand_tracker.history_len))
        print('latest_measurement: {}, {}'.format(
            new_time.to_sec(), hand_tracker.last_measure.to_sec()))
