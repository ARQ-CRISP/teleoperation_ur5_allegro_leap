#! /usr/bin/env python
import rospy
import numpy as np
from utils import list2ROSPose

import tf
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped


class UR5_State(object):

    def __init__(self, tf_buffer, base_frame="world", ee_frame="hand_root", time=None):
        time = rospy.Time.now() if time is None else time

        self.last_time_measured = None
        self.__last_time_moved = None
        self.__last_movement = None
        self.base_frame = base_frame
        self.ee_frame = ee_frame
        self.name = 'UR5 ARM'
        self.buffer = tf_buffer
        self.__ee_position, self.__ee_orientation = self.__measure_current_state(time)
        rospy.loginfo(rospy.get_name().split('/')[1] + ': --> Initialization of ' + self.name +
                      " Completed! Measured at: " + str(self.last_time_measured.to_sec()))

    @property
    def last_movement(self):
        return (self.__last_time_moved, self.__last_movement)
    
    @property
    def ee_pose(self):
        return [self.__ee_position.tolist(), self.__ee_orientation.tolist()]

    @ee_pose.setter
    def ee_pose(self, pose):
        self.ee_position = pose[0]
        self.ee_orientation = pose[1]

    @property
    def ee_position(self):
        return self.__ee_position.tolist()

    @ee_position.setter
    def ee_position(self, ee_position):
        self.__ee_position = ee_position if ee_position is np.array else np.array(
            ee_position)
        self.last_time_measured = rospy.Time.now()
        rospy.logdebug(rospy.get_name().split()[0] + '--> Position of ' + self.name +
                      " set at " + str(self.__ee_position.round(3)) +
                      " Completed at: " + str(self.last_time_measured.to_sec()))

    @property
    def ee_orientation(self):
        return self.__ee_orientation.tolist()

    @ee_orientation.setter
    def ee_orientation(self, ee_orientation):
        self.__ee_orientation = ee_orientation if ee_orientation is np.array else np.array(
            ee_orientation)
        self.last_time_measured = rospy.Time.now()
        rospy.logdebug(rospy.get_name().split()[0] + '--> Orientation of ' + self.name +
                      " set at " + str(self.__ee_orientation.round(3)) +
                      " Completed at: " + str(self.last_time_measured.to_sec()))


    def to_PoseStamped(self, time=None, stamped=True):
        if time is None:
            time = self.last_time_measured if self.__last_time_moved is None or\
                (self.last_time_measured > self.__last_time_moved) else self.__last_time_moved
        postion = self.__ee_position.round(4).tolist()
        orientation = self.__ee_orientation.round(4).tolist()
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
        rospy.logdebug(rospy.get_name().split()[0] + '--> movement of ' + self.name +
                      " by " + str(self.__last_movement.round(3)) +
                      " Completed at: " + str(self.__last_time_moved.to_sec()))
        return self

    def update_measure(self, time=None):
        time = rospy.Time.now() if time is None else time
        self.ee_position, self.ee_orientation = self.__measure_current_state(
            time)

        rospy.logdebug(rospy.get_name() + '--> Update of ' + self.name +
                      " Completed! Measured at: " + str(self.last_time_measured.to_sec()))

    def __measure_current_state(self, time=None, finger_section=-1):

        time = rospy.Time.now() if time is None else time
        # self.buffer.can_transform(
            # self.base_frame, self.ee_frame, time, rospy.Duration(0.2))
        root2link = self.buffer.lookup_transform(
            self.base_frame, 
            self.ee_frame,
            time, rospy.Duration(0.2))
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
    ur5_state = UR5_State(tf_buffer)
    # index_state = Allegro_Finger_State('Index', tf_buffer)

    pprint(ur5_state.ee_pose)
    ur5_state.translate_by([1, 1, 1])
    pprint(ur5_state.ee_pose)
    ur5_state.ee_pose = [[0,0,0],[0,0,0,1]] 
    pprint(ur5_state.ee_pose)
    pprint(ur5_state.to_PoseStamped(stamped=False))

    pprint(ur5_state.last_movement)
