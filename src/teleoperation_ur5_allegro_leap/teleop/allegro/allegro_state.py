#! /usr/bin/env python
import rospy
import numpy as np
from utils import list2ROSPose

import tf
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
from allegro_utils import allegro_finger2linklist, finger_allegro_idx, allegro_fingers


class Allegro_State(object):

    finger_names = allegro_fingers  # ['Index', 'Middle', 'Ring',  'Thumb']
    def __init__(self, tf_buffer, base_frame="hand_root", time=None):

        time = rospy.Time.now() if time is None else time
        self.fingers = dict()

        for finger in self.finger_names:
            self.fingers[finger] = Allegro_Finger_State(
                finger, tf_buffer, time, base_frame)

    @property
    def last_movements(self):
        movements = []
        for finger_name, finger in self.fingers.items():
            movements.append(finger.last_movement)
        return movements

    @property
    def ee_poses(self):
        poses = []
        for finger in self.fingers:
            poses.append(self.fingers[finger].ee_pose)
        return poses

    @ee_poses.setter
    def ee_poses(self, poses):
        for finger_name, pose in zip(self.finger_names, poses):
            self.fingers[finger_name] = pose

    def to_target_dict(self):
        targets = dict()
        for finger_names, finger_state in self.fingers.items():
            targets.update(finger_state.to_target_dict())
        return targets

    def to_PoseStamped(self, stamped=True):
        posestamped = []
        for finger_names, finger_state in self.fingers.items():
            posestamped.append(finger_state.to_PoseStamped(stamped=stamped))
        return posestamped

    def translate_by(self, translations, time=None):
        for finger_name, translation in zip(self.finger_names, translations):
            self.fingers[finger_name].translate_by(translation, time)

    def update_measures(self, time=None):
        # time = rospy.Time.now() if time is None else time
        for finger_name in self.finger_names:
            self.fingers[finger_name].update_measure(time)


class Allegro_Finger_State():

    def __init__(self, finger_name, tf_buffer, time=None, base_frame="hand_root", logging=False):

        time = rospy.Time.now() if time is None else time
        self.__finger2linklist = None
        self.last_time_measured = None
        self.__last_time_moved = None
        self.__last_movement = None
        self.base_frame = base_frame
        self.name = finger_name
        self.buffer = tf_buffer
        self.logging = logging
        self.ee_position, self.ee_orientation = self.__measure_current_state(
            time)
        rospy.loginfo(rospy.get_name() + '--> Initialization of ' + self.name +
                      " Completed! Measured at: " + str(self.last_time_measured.to_sec()))

    @property
    def last_movement(self):
        return (self.__last_time_moved, self.__last_movement)

    @property
    def ee_pose(self):
        return [self.ee_position.tolist(), self.ee_orientation.tolist()]

    @ee_pose.setter
    def ee_pose(self, ee_pose):
        self.ee_position = ee_pose[0] if ee_pose[0] is np.array else np.array(
            ee_pose[0])
        self.ee_orientation = ee_pose[1] if ee_pose[1] is np.array else np.array(
            ee_pose[1])

    def to_target_dict(self, rotation=False):
        target = dict()
        allegro_finger_str = "f" + str(finger_allegro_idx[self.name])
        position = self.ee_position.tolist()
        orientation = self.ee_orientation.tolist()

        pos = {"x": position[0], "y": position[1], "z": position[2]}
        target['pos'] = pos
        if rotation:
            rot = {"x": orientation[0], "y": orientation[1],
                   "z": orientation[2], "w": orientation[3]}
            target.update({"rot": rot})

        return {allegro_finger_str: target}

    def to_PoseStamped(self, time=None, stamped=True):
        if time is None:
            time = self.last_time_measured if self.__last_time_moved is None or\
                (self.last_time_measured > self.__last_time_moved) else self.__last_time_moved
        postion = self.ee_position.round(4).tolist()
        orientation = self.ee_orientation.round(4).tolist()
        stamped = PoseStamped()
        stamped.header.stamp = time
        stamped.header.frame_id = self.base_frame
        stamped.pose.position.x = postion[0]
        stamped.pose.position.y = postion[1]
        stamped.pose.position.z = postion[2]

        stamped.pose.orientation.x = orientation[0]
        stamped.pose.orientation.y = orientation[1]
        stamped.pose.orientation.z = orientation[2]
        stamped.pose.orientation.w = orientation[3]

        if stamped:
            return stamped
        else:
            return stamped.pose

    def translate_by(self, translation, time=None):
        if translation is not np.array:
            translation = np.array(translation)

        self.ee_position += translation
        self.__last_movement = translation
        self.__last_time_moved = rospy.Time.now() if time is None else time
        if self.logging:
            rospy.loginfo(rospy.get_name().split()[0] + '--> movement of ' + self.name +
                        " by " + str(self.__last_movement.round(3)) +
                        " Completed at: " + str(self.__last_time_moved.to_sec()))
        return self

    def update_measure(self, time=None):
        time = rospy.Time.now() if time is None else time
        self.ee_position, self.ee_orientation = self.__measure_current_state(
            time)
        if self.logging:
            rospy.loginfo(rospy.get_name() + '--> Update of ' + self.name +
                        " Completed! Measured at: " + str(self.last_time_measured.to_sec()))

    def update_orient(self, time=None):
        time = rospy.Time.now() if time is None else time
        _, self.ee_orientation = self.__measure_current_state(
            time)
        if self.logging:
            rospy.loginfo(rospy.get_name() + '--> Update of ' + self.name +
                        " Completed! Measured at: " + str(self.last_time_measured.to_sec()))

    def __measure_current_state(self, time=None, finger_section=-1):

        time = rospy.Time.now() if time is None else time

        link = allegro_finger2linklist[self.name][finger_section]
        root2link = self.buffer.lookup_transform(
            self.base_frame, link, time, rospy.Duration(0.2))
        position = np.array([
            root2link.transform.translation.x,
            root2link.transform.translation.y,
            root2link.transform.translation.z])
        orientation = np.array([
            root2link.transform.rotation.x,
            root2link.transform.rotation.y,
            root2link.transform.rotation.z,
            root2link.transform.rotation.w
        ])
        self.last_time_measured = time
        return [position, orientation]


if __name__ == "__main__":
    from pprint import pprint
    rospy.init_node('test_state')
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100), True)
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(rospy.Duration(.5))
    allegro_state = Allegro_State(tf_buffer)
    # index_state = Allegro_Finger_State('Index', tf_buffer)

    pprint(allegro_state.ee_poses)

    pprint(allegro_state.to_target_dict())
    pprint(allegro_state.to_PoseStamped(False))

    pprint(allegro_state.last_movements)
