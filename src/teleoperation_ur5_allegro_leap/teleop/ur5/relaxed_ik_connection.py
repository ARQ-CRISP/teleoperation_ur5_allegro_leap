#! /usr/bin/env python
from __future__ import print_function, division, absolute_import
import numpy as np
import sys

import rospy
# from visualization_msgs.msg import Marker
from PyKDL import Frame, Rotation, Vector
from tf_conversions import posemath as pm
from moveit_commander import MoveGroupCommander, roscpp_initialize
from moveit_commander.conversions import list_to_pose, pose_to_list

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from relaxed_ik.msg import EEPoseGoals, JointAngles

from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix


class Relaxed_UR5_Connection():
    
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    joint_states_topic = '/move_group/fake_controller_joint_states'
    absolute_teleop_mode_rosparam = ur5_teleop_prefix + 'teleop_mode_absolute'
    
    relaxed_ik_pose_goals_topic = '/relaxed_ik/ee_pose_goals'
    relaxed_ik_solutions_topic = '/relaxed_ik/joint_angle_solutions'
    
    request_topic = ur5_teleop_prefix + 'ur5_pose_targets'
    marker_topic = ur5_teleop_prefix + 'target_marker_debug'
    
    def __init__(self, init_state=[0., 0., 0., 0., 0., 0.], group_name='ur5_arm', debug_mode=False):
        
        self.jangles = JointState()
        self.jangles.name = self.jnames
        self.bias = Frame(Rotation.Quaternion(0.5, 0.5, 0.5, 0.5), Vector())
        self.posegoal = EEPoseGoals()
        self.posegoal.ee_poses.append(Pose())
        self.set_absolute_mode_flag(self.get_absolute_mode_flag()) # sets to default value
        self.joint_target_pub = rospy.Publisher(self.joint_states_topic, JointState, queue_size=1)
        self.goal_pub = rospy.Publisher(self.relaxed_ik_pose_goals_topic, EEPoseGoals, queue_size=10)
        self.solution_sub = rospy.Subscriber(self.relaxed_ik_solutions_topic, JointAngles, self.OnSolutionReceived, queue_size=10)
        self.ur5_target_subscriber = None
        
        self.debug_mode = debug_mode
        roscpp_initialize(sys.argv)
        self.moveit_interface = MoveGroupCommander(group_name)
        self.moveit_interface.set_end_effector_link('hand_root')
        # print(self.moveit_interface.get_end_effector_link())
        self.moveit_interface.go(joints=init_state, wait=True) #go to the initial position requested to relaxed_ik
        self.OnSolutionReceived(JointAngles(angles=Float64(init_state)))
        pose = self.moveit_interface.get_current_pose().pose #get the pose relative to world
        # self.init_pose = np.array(pose_to_list(pose))
        self.init_pose = pm.fromMsg(pose)
        # 0.7071446353719505, 0., -0.7071446353719505, 0.
        # 0.7071446353719505, 0., -0.7071446353719505, 0.
        # self.bias = Frame(Rotation.Quaternion(0, 0, 0.7071068, 0.707106), Vector())
        
    
    def set_debug_properties(self):
        self.marker_target_pub = rospy.Publisher(self.marker_topic, PoseStamped, queue_size=1)
        self.target_marker = Marker()
        self.target_marker.type = Marker.ARROW
        self.target_marker.scale.x, self.target_marker.scale.y, self.target_marker.scale.z = 0.2, 0.01, 0.01
        self.target_marker.color.b = self.target_marker.color.a = 1.0
        self.target_marker.header.frame_id = 'world'
    
    def get_absolute_mode_flag(self):
        return rospy.get_param(self.absolute_teleop_mode_rosparam, default=True)
    
    def set_absolute_mode_flag(self, value):
        if value==True:
            rospy.loginfo('{}: Receiving targets in world frame!'.format(rospy.get_name()))
        else:
            rospy.loginfo('{}: Receiving targets in in retation to the initial position!'.format(rospy.get_name()))
        rospy.set_param(self.absolute_teleop_mode_rosparam, value)
    
    def goto_pose(self, goal_pose):
        """[summary]

        Args:
            goal_pose (numpy.array): array containing the position and orientation relative to the initial position
            of the goal.
        """
        posegoal = EEPoseGoals()
        self.posegoal.header.stamp = rospy.Time.now()
        self.posegoal.ee_poses[0] = pm.toMsg(goal_pose)
        self.goal_pub.publish(self.posegoal)
        
            
    def send_marker(self, pose, frame='world'):
        self.target_marker.pose = pose
        tgt = PoseStamped(pose=pose)
        tgt.header.frame_id = frame
        self.marker_target_pub.publish(tgt)
        
    def pose_to_world_frame(self, relative_pose):
        # init_f = self.init_pose
        # relative_f = relative_pose
        world_f = self.init_pose * relative_pose #* self.bias)
        # world_pose = list(world_f.p) + list(world_f.M.GetQuaternion())
        
        return world_f
    
    def pose_to_relative_frame(self, world_pose):
        world_f = world_pose
        
        
        init_f = self.init_pose
        init_2_target_f = (self.init_pose.Inverse() * world_pose) #* self.bias.Inverse()
        return init_2_target_f
        
    def OnSolutionReceived(self, joint_angle_msg):
        """Function activated once the relaxedIK module has a ready solution

        Args:
            joint_angle_msg (relaxed_ik.msg.JointAngles): The JointAngles solution message
        """
        # print(joint_angle_msg.angles.data)
        self.jangles.position = joint_angle_msg.angles.data
        self.joint_target_pub.publish(self.jangles)
        
    def OnPoseRequest(self, pose_stamped):
        request = pm.fromMsg(pose_stamped.pose)
        
        if self.get_absolute_mode_flag():
            request = self.pose_to_relative_frame(request)
        
        if self.debug_mode:
            
            self.send_marker(
                pm.toMsg(self.correct_bias(request)), 'hand_root')
            
        self.goto_pose(
            self.correct_bias(request)
            )
     
    def correct_bias(self, frame):
        bb = Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0 ,0)
        # tgt = Frame(Rotation.Quaternion(0.5, 0.5, 0.5, -0.5)).Inverse() * Frame(bb) * frame * Frame(bb).Inverse()
        return Frame(bb) * frame * Frame(bb).Inverse()
    
    def listen(self):
        if self.ur5_target_subscriber is not None:
            self.ur5_target_subscriber.close()
            self.ur5_target_subscriber = None
        self.ur5_target_subscriber = rospy.Subscriber(self.request_topic, PoseStamped, self.OnPoseRequest, queue_size=10)
        self.set_debug_properties()