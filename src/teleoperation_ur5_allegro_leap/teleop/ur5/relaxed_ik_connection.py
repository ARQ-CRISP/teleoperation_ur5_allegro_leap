#! /usr/bin/env python
from __future__ import print_function, division, absolute_import
import numpy as np
import sys

import rospy
import actionlib

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

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix


class Relaxed_UR5_Connection():
    
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    joint_states_topic = '/move_group/fake_controller_joint_states'
    real_robot_action_server = 'follow_joint_trajectory'
    absolute_teleop_mode_rosparam = ur5_teleop_prefix + 'teleop_mode_absolute'
    
    relaxed_ik_pose_goals_topic = '/relaxed_ik/ee_pose_goals'
    relaxed_ik_solutions_topic = '/relaxed_ik/joint_angle_solutions'
    
    request_topic = ur5_teleop_prefix + 'ur5_pose_targets'
    marker_topic = ur5_teleop_prefix + 'target_marker_debug'
    
    def __init__(self, init_state=[0., 0., 0., 0., 0., 0.], movegroup='ur5_arm', sim=True, debug_mode=False):
        self.sim = sim
        self.jangles = JointState()
        self.jangles.name = self.jnames
        
        self.rotation_bias = Frame(Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0 ,0))
        
        self.posegoal = EEPoseGoals()
        self.posegoal.ee_poses.append(Pose())
        self.set_absolute_mode_flag(self.get_absolute_mode_flag()) # sets to default value
        
        self.ur5_target_subscriber = None
        self.joint_target_pub = None
        self.set_controller_driver_connection()
        self.goal_pub = rospy.Publisher(self.relaxed_ik_pose_goals_topic, EEPoseGoals, queue_size=10)
        self.solution_sub = rospy.Subscriber(self.relaxed_ik_solutions_topic, JointAngles, self.OnSolutionReceived, queue_size=10)

        self.debug_mode = debug_mode
        roscpp_initialize(sys.argv)
        self.moveit_interface =  movegroup if isinstance(movegroup, MoveGroupCommander) else MoveGroupCommander(movegroup)
        # self.moveit_interface.set_end_effector_link('hand_root')
        # print(self.moveit_interface.get_end_effector_link())
        self.moveit_interface.go(joints=init_state, wait=True) #go to the initial position requested to relaxed_ik
        self.OnSolutionReceived(JointAngles(angles=Float64(init_state)))
        pose = self.moveit_interface.get_current_pose(end_effector_link="hand_root").pose #get the pose relative to world
        
        self.init_pose = pm.fromMsg(pose)
        

    def set_controller_driver_connection(self):
        if not self.sim:
            self.joint_target_pub = actionlib.SimpleActionClient(self.real_robot_action_server, FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.joint_target_pub.wait_for_server()
            print("Connected to server") 
        else:
            self.joint_target_pub = rospy.Publisher(self.joint_states_topic, JointState, queue_size=1)


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
        world_f = self.init_pose * relative_pose 
        return world_f
    
    def pose_to_relative_frame(self, world_pose):
        # world_f = world_pose
        # init_f = self.init_pose
        init_2_target_f = (self.init_pose.Inverse() * world_pose) 
        return init_2_target_f
        
    def OnSolutionReceived(self, joint_angle_msg):
        """Function activated once the relaxedIK module has a ready solution

        Args:
            joint_angle_msg (relaxed_ik.msg.JointAngles): The JointAngles solution message
        """
        # print(joint_angle_msg.angles.data)
        if self.sim:
            self.jangles.position = joint_angle_msg.angles.data
            self.joint_target_pub.publish(self.jangles)
        else:
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.jnames
            g.trajectory.points = [
                JointTrajectoryPoint(
                    positions=joint_angle_msg.angles.data,
                    velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
            self.joint_target_pub.send_goal(g)
            try:
                self.joint_target_pub.wait_for_result()
            except KeyboardInterrupt:
                self.joint_target_pub.cancel_goal()
                raise
        
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
        """Corrects the rotation bias of the targets

        Args:
            frame (PyKDL.Frame): Frame of the target

        Returns:
            corrected_frame (PyKDL.Frame): Frame with corrected_bias
        """
        # tgt = Frame(Rotation.Quaternion(0.5, 0.5, 0.5, -0.5)).Inverse() * Frame(bb) * frame * Frame(bb).Inverse()
        return  self.rotation_bias * frame * self.rotation_bias.Inverse()
    
    def listen(self):
        if self.ur5_target_subscriber is not None:
            self.ur5_target_subscriber.close()
            self.ur5_target_subscriber = None
        self.ur5_target_subscriber = rospy.Subscriber(self.request_topic, PoseStamped, self.OnPoseRequest, queue_size=10)
        self.set_debug_properties()