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
import math
import numpy as np
import sys

import rospy
import roslaunch
import rospkg

# from relaxed_wrapper import Relaxed_UR5_Controller
import moveit_commander
from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import list_to_pose, list_to_pose_stamped, transform_to_list, pose_to_list, list_to_transform
from moveit_msgs.msg._RobotTrajectory import RobotTrajectory
from trajectory_msgs.msg._JointTrajectory import JointTrajectory
from trajectory_msgs.msg._JointTrajectoryPoint import JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped, Vector3
import tf
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from leap_motion.msg import leapros, Human, Hand, Finger, Bone
# from abc import ABCMeta, abstractmethod
from utils import pose2str, STransform2SPose, generate_pose_marker, list2ROSPose, ROSPose2list
from ur5_state import UR5_State
from ur5_utils import combine_rotations, get_qrotation
from trac_ik_python.trac_ik import IK
from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_leap_teleop.config import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame, config_file_name

# Class responsible of defining the reading of the leapmotion data topic and providing it to the main app
class Leap_Teleop_UR5_moveit(object):

    leap_wrist_tf = 'leap_hand_wrist'

    def __init__(self, leap_topic='/leap_motion/leap_device', lefthand_mode=False, groupname='ur5_arm'):
        # init_time = rospy.Time().now()
        self.latest_teleop_state = None
        self.leap_topic = leap_topic
        self.UR5group = MoveGroupCommander(groupname)  # instantiate the commander object
        self.UR5group.set_planning_time(1.)
        self.UR5group.allow_replanning(True)
        self.UR5group.set_num_planning_attempts(5)
        target = self.UR5group.get_named_target_values('start')
        self.UR5group.go(target)
        self.IK_solver = IK('world', 'hand_root', epsilon=1e-05, timeout=0.01)
        # self.UR5group.set

        self.relaxed_solver = RelaxedIK.init_from_config(config_file_name)
        self.lefthand_mode = lefthand_mode
        rospy.loginfo(rospy.get_name().split('/')[1] + ': Initialization....')
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(50))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.5)
        self.UR5_state = UR5_State(self.tf_buffer, base_frame='world', ee_frame='hand_root')

        self.marker_pub = rospy.Publisher(
            'ur5_teleop_debug', MarkerArray, queue_size=5)

        self.leap_listener = rospy.Subscriber(
            self.leap_topic, Human, self.__OnLeapMessageReceived, queue_size=1)

    @property
    def leap_hand_str(self):
        return "left_" if self.lefthand_mode else "right_"

    @property
    def finger_names(self):
        return ["Thumb", "Index", "Middle", "Ring", "Pinky"]

    
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
                # self.tf_buffer.can_transform()
                source = self.UR5_state.to_PoseStamped().pose
                real_source = self.UR5group.get_current_pose().pose
                time_interval = new_teleop_state['time'] - self.latest_teleop_state['time'] if self.latest_teleop_state is not None else rospy.Duration(.1)
                translation, hand_quat = self.set_target_on_velocity(new_teleop_state)
                
                target = self.UR5_state.to_PoseStamped()
                # print(["%1.3f" %e for e in ee_orientation], ["%1.3f" %s  for s in self.UR5_state.ee_pose[1]], sep=' ---> ')
                m1 = generate_pose_marker(real_source, Marker.ARROW, scale=[.01,.01,.02], ref_frame='common_world', RGBA= [.7, .0, .4, 0.7],name_space='real_Source')
                markers.markers.append(m1)
                m2 = generate_pose_marker(source, Marker.ARROW, scale=[.01,.01,.02], ref_frame='common_world', RGBA= [1.,1.,1.,0.5],name_space='source')
                markers.markers.append(m2)
                m3 = generate_pose_marker(target.pose, Marker.ARROW, scale=[.01,.01,.02], ref_frame='common_world', RGBA= [.0, .5, .5, 0.7],name_space='target')
                markers.markers.append(m3)
                pose = pose_to_list(target.pose)
                # hand_quat = pose[3:]
                xopt = self.relaxed_solver.solve([translation], [hand_quat])
                j_list = joint_state_define(xopt)
                current_j = self.UR5group.get_current_joint_values()
                # j_list = self.IK_solver.get_ik(current_j, *(pose[0]+pose[1]))
                # for attempt in range(10):
                #     j_list = self.IK_solver.get_ik(current_j + np.random.uniform(-1/2, 1/2, 6)
                #         , *(pose[0]+pose[1]))
                #     if j_list is not None:
                #         break
                # print('solving attempts: {}'.format(attempt+1))
                # print(j_list)
                if j_list is not None:
                    # self.UR5group.set_joint_value_target(j_list)
                    # print(ROSPose2list((target.pose)))
                    # self.UR5group.set_pose_target(target)
                    # self.UR5group.go(wait=True)
                    # plan = self.UR5group.plan()
                    if np.linalg.norm(np.array(current_j) - j_list.position, 2) >=1e-3:
                        plan = self.generate_poly_trajectory(current_j, j_list.position.tolist(), time_interval)
                        self.UR5group.execute(plan, wait=True)
                    # print(len(plan.joint_trajectory.points))
                    # (plan, fraction) = self.UR5group.compute_cartesian_path([source, target.pose], eef_step=0.01, jump_threshold=0.05)
                    # self.group.stop()
                    # self.group.set_pose_target(target.pose)
                    # plan = self.group.go(wait=False)
                    # self.publish_goals(new_teleop_state, markers)
                else:
                    print('Error Solving the position')
            except (tf.LookupException, tf.ExtrapolationException) as e:
                print(e)
            self.marker_pub.publish(markers)
        else:
            # Reset the state of the teleop otherwhise I am getting peaks in the derivative
            self.latest_teleop_state = None

    def generate_poly_trajectory(self, J_start, J_end, duration, order=1):
        t = np.linspace(0, duration.to_sec() + .5, 10)
        plan = RobotTrajectory()
        plan.joint_trajectory = JointTrajectory()
        plan.joint_trajectory.joint_names = self.UR5group.get_active_joints()
        poly_position = [np.poly1d(np.polyfit([0., duration.to_sec()], [s, tar], order)) for s,tar in zip(J_start, J_end)]
        
        positions = np.array([p(t) for p in poly_position]);\
        velocities = np.array([np.concatenate([np.polyder(p)(t)]) for p in poly_position])
        accellerations = np.array([np.polyder(p, 2)(t) for p in poly_position])
        # accellerations = np.zeros(velocities.shape)
        # accellerations[:,0] = velocities[:, 1] - velocities[:, 0]
        # accellerations[:,-1] = velocities[:, -1] - velocities[:, -2]

        for i in range(len(t)):
            traj_point = JointTrajectoryPoint(
                time_from_start=rospy.Duration(t[i]),
                positions=positions[:,i].tolist(), 
                velocities=velocities[:,i].tolist(), 
                accelerations=accellerations[:,i].tolist())
            plan.joint_trajectory.points.append(traj_point)

        return plan
        

    def set_target_on_velocity(self, current_teleop_state):
        Stransform_leap = self.tf_buffer.lookup_transform('world', 
            self.leap_hand_str + self.leap_wrist_tf, 
            current_teleop_state['time'], 
            rospy.Duration(2.0))
        hand_pose = STransform2SPose(Stransform_leap)
        current_teleop_state['leap'] = hand_pose.pose
        last_state = self.latest_teleop_state
        self.latest_teleop_state = current_teleop_state
        if last_state is not None:
            current_leap_pose = pose_to_list(hand_pose.pose)
            last_leap_pose = pose_to_list(last_state['leap'])
            translation = np.array(current_leap_pose[:3]) - np.array(last_leap_pose[:3])
            if np.linalg.norm(translation) > 0.002:
                # print(translation.round(3), np.linalg.norm(translation).round(3), np.array(current_leap_pose[1]).round(3), sep=' - ')
                
                self.UR5_state.translate_by(translation)
                self.UR5_state.ee_orientation = current_leap_pose[3:]
            return translation, self.UR5_state.ee_orientation
        else:
            return np.zeros((3,)), self.UR5_state.ee_orientation



if __name__ == '__main__':
    rospy.init_node('UR5_teleop')
    moveit_commander.roscpp_initialize(sys.argv)
    Leap_listener = Leap_Teleop_UR5_moveit(leap_topic='/leap_motion/leap_filtered')
    rospy.spin()
