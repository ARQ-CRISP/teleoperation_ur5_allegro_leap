#! /usr/bin/env python
from __future__ import print_function, division, absolute_import
import numpy as np
from copy import deepcopy
import sys

import rospy

# from visualization_msgs.msg import Marker
from PyKDL import Frame, Rotation, Vector
from tf_conversions import posemath as pm
from moveit_commander import MoveGroupCommander, roscpp_initialize
from moveit_commander.conversions import list_to_pose, pose_to_list

from sensor_msgs.msg import JointState
# from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from relaxed_ik.msg import EEPoseGoals, JointAngles

# from teleoperation_ur5_allegro_leap.teleop.ur5 import WS_Bounds
from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix
from teleoperation_ur5_allegro_leap.teleop.ur5 import JointMovementManager
from teleoperation_ur5_allegro_leap.teleop.ur5.arm_teleop_input import Arm_Teleop_Input, Combined_Arm_Teleop_Input


class Relaxed_UR5_Connection():
    
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    absolute_teleop_mode_rosparam = ur5_teleop_prefix + 'teleop_mode_absolute'
    
    relaxed_ik_pose_goals_topic = '/relaxed_ik/ee_pose_goals'
    relaxed_ik_solutions_topic = '/relaxed_ik/joint_angle_solutions'
    
    request_topic = ur5_teleop_prefix + 'ur5_pose_targets'
    goal_marker_topic = ur5_teleop_prefix + 'target_marker_debug'
    
    max_angle = np.pi/8.
    min_angle = 1e-4
    
    def __init__(self, init_state=[0., 0., 0., 0., 0., 0.],
                movegroup='ur5_arm', sim=True, debug_mode=False):
        
        self.sim = sim
        self.safety_counter = 0
        
        self.rotation_bias = Frame(Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0 ,0))
        
        self.posegoal = EEPoseGoals()
        self.posegoal.ee_poses.append(Pose())
        self.set_absolute_mode_flag(self.get_absolute_mode_flag()) # sets to default value
        
        
        self.joint_manager = JointMovementManager.generate_manager(init_state, sim=sim)
        
        self.solution_sub = rospy.Subscriber(
            self.relaxed_ik_solutions_topic, JointAngles, self.OnSolutionReceived, queue_size=1)


        roscpp_initialize(sys.argv)
        self.moveit_interface = movegroup if isinstance(movegroup, MoveGroupCommander) else MoveGroupCommander(movegroup)
        self.moveit_interface.go(
            joints=JointState(name=self.jnames, position=init_state),
            wait=True) #go to the initial position requested to relaxed_ik
        
        self.OnSolutionReceived(
            JointAngles(angles=Float64(init_state)))
        
        pose = self.moveit_interface.get_current_pose(end_effector_link="hand_root").pose #get the pose relative to world
        
        #Input manager Initialised here
        self.input_manager = Combined_Arm_Teleop_Input(pm.fromMsg(pose))
        # self.init_pose = pm.fromMsg(pose)
        
        rospy.Timer(rospy.Duration(1/15.), lambda msg: self.check_ee_safety())


    def set_debug_properties(self):
        self.marker_target_pub = rospy.Publisher(self.goal_marker_topic, PoseStamped, queue_size=1)
        self.target_marker = Marker(type=Marker.ARROW)
        # self.target_marker.type = Marker.ARROW
        self.target_marker.scale.x, self.target_marker.scale.y, self.target_marker.scale.z = 0.2, 0.01, 0.01
        self.target_marker.color.b = self.target_marker.color.a = 1.0
    
    def get_absolute_mode_flag(self):
        return rospy.get_param(self.absolute_teleop_mode_rosparam, default=True)
    
    def set_absolute_mode_flag(self, value):
        if value==True:
            rospy.loginfo('{}: Receiving targets in world frame!'.format(rospy.get_name()))
        else:
            rospy.loginfo('{}: Receiving targets in in retation to the initial position!'.format(rospy.get_name()))
        rospy.set_param(self.absolute_teleop_mode_rosparam, value)
    
        
    def OnSolutionReceived(self, joint_angle_msg):
        """Function activated once the relaxedIK module has a ready solution

        Args:
            joint_angle_msg (relaxed_ik.msg.JointAngles): The JointAngles solution message
        """
        # print(joint_angle_msg.angles.data)
        diff = (
            np.asarray(joint_angle_msg.angles.data) - \
                np.asarray(self.joint_manager.current_j_state.position)).round(2)
        if self.min_angle < np.max(diff) < self.max_angle :
            self.joint_manager.generate_movement(joint_angle_msg.angles.data)
            rospy.loginfo(str(diff))
            
        elif np.any(np.absolute(diff) >= self.max_angle):
            rospy.logwarn('Arm seems to move too fast! Difference of sequencial joint angles too damn high!')
            
    def check_ee_safety(self):
        
        pose = self.moveit_interface.get_current_pose(end_effector_link="hand_root").pose
        frame = pm.fromMsg(pose)
        
        if not self.input_manager.workspace.in_bounds(frame.p, 5e-2):
            self.safety_counter += 1
            rospy.logwarn('EE Out of Bounds! Return to Workspace!')
        else:
            self.safety_counter = 0
            
        if self.safety_counter > 20:
            rospy.logwarn('EE Out of Bounds! Emergency Stop!')
            self.joint_manager.emergency_stop()
        

    def on_kill(self):
        if self.joint_manager is not None:
            self.joint_manager.terminate()
            
        