#! /usr/bin/env python
from __future__ import print_function, division, absolute_import
import numpy as np
from copy import deepcopy
import sys

import rospy
import actionlib

# from visualization_msgs.msg import Marker
from PyKDL import Frame, Rotation, Vector
from tf_conversions import posemath as pm
from moveit_commander import MoveGroupCommander, roscpp_initialize
from moveit_commander.conversions import list_to_pose, pose_to_list

from sensor_msgs.msg import JointState
# from moveit_msgs.ms+g import RobotState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from relaxed_ik.msg import EEPoseGoals, JointAngles

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from teleoperation_ur5_allegro_leap.teleop.ur5 import WS_Bounds
from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix



from std_msgs.msg import Int32

# jorder = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# realj = [2.3744237422943115, -2.0752771536456507, -1.7465012709247034, -0.8918698469745081, 1.5678939819335938, 0.013490866869688034]

class Relaxed_UR5_Connection():
    
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    joint_states_topic = '/move_group/fake_controller_joint_states'
    real_robot_action_server = '/scaled_pos_joint_traj_controller/follow_joint_trajectory'
    absolute_teleop_mode_rosparam = ur5_teleop_prefix + 'teleop_mode_absolute'
    
    relaxed_ik_pose_goals_topic = '/relaxed_ik/ee_pose_goals'
    relaxed_ik_solutions_topic = '/relaxed_ik/joint_angle_solutions'
    
    request_topic = ur5_teleop_prefix + 'ur5_pose_targets'
    goal_marker_topic = ur5_teleop_prefix + 'target_marker_debug'
    workspace_marker_topic = ur5_teleop_prefix + 'workspace'
    time_to_target = 1 / 15.
    
    def __init__(self, init_state=[0., 0., 0., 0., 0., 0.],
                workspace = WS_Bounds(-np.infty * np.ones(3), np.infty * np.ones(3)),
                movegroup='ur5_arm', sim=True, debug_mode=False):
        self.sim = sim
        self.jstate_buffer = [] if sim else FollowJointTrajectoryGoal(
            trajectory=JointTrajectory(joint_names=self.jnames))
        self.jangles = JointState(name=self.jnames, position=init_state)
        # self.__set_trajectory_buffer(init_state)
        self.debug_buffer_size_pub = rospy.Publisher('/buffer_size', Int32, queue_size=1)
        self.add_to_buffer(init_state)
        self.workspace = workspace
        
        self.rotation_bias = Frame(Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0 ,0))
        
        self.posegoal = EEPoseGoals()
        self.posegoal.ee_poses.append(Pose())
        self.set_absolute_mode_flag(self.get_absolute_mode_flag()) # sets to default value
        
        self.ur5_target_subscriber = None
        self.joint_target_pub = None
        self.set_controller_driver_connection()
        self.goal_pub = rospy.Publisher(self.relaxed_ik_pose_goals_topic, EEPoseGoals, queue_size=10)
        self.solution_sub = rospy.Subscriber(
            self.relaxed_ik_solutions_topic, JointAngles, self.OnSolutionReceived, queue_size=1)

        self.debug_mode = debug_mode
        roscpp_initialize(sys.argv)
        self.moveit_interface =  movegroup if isinstance(movegroup, MoveGroupCommander) else MoveGroupCommander(movegroup)
        
        self.moveit_interface.go(joints=self.jangles, wait=True) #go to the initial position requested to relaxed_ik
        self.OnSolutionReceived(JointAngles(angles=Float64(init_state)))
        pose = self.moveit_interface.get_current_pose(end_effector_link="hand_root").pose #get the pose relative to world
        
        
        self.init_pose = pm.fromMsg(pose)

            
    def set_controller_driver_connection(self):
        if not self.sim:
            self.joint_target_pub = actionlib.SimpleActionClient(self.real_robot_action_server, FollowJointTrajectoryAction)
            rospy.loginfo("[" + rospy.get_name() + "]" + " Waiting for server...")
            self.joint_target_pub.wait_for_server()
            rospy.loginfo("[" + rospy.get_name() + "]" + " Connected to Robot!") 
        else:
            self.joint_target_pub = rospy.Publisher(self.joint_states_topic, JointState, queue_size=1)


    def set_debug_properties(self):
        self.marker_target_pub = rospy.Publisher(self.goal_marker_topic, PoseStamped, queue_size=1)
        self.marker_workspace_pub = rospy.Publisher(self.workspace_marker_topic, Marker, queue_size=1)
        self.workspace_marker = Marker(type=Marker.CUBE)
        self.target_marker = Marker(type=Marker.ARROW)
        # self.target_marker.type = Marker.ARROW
        self.target_marker.scale.x, self.target_marker.scale.y, self.target_marker.scale.z = 0.2, 0.01, 0.01
        self.target_marker.color.b = self.target_marker.color.a = 1.0
        
        ws_center, ws_scale = self.workspace.get_center_scale()
        print(ws_center, ws_scale)
        self.workspace_marker.header.frame_id = self.target_marker.header.frame_id = 'world'
        self.workspace_marker.pose.position.x, self.workspace_marker.pose.position.y, self.workspace_marker.pose.position.z = ws_center
        self.workspace_marker.pose.orientation.w = 1.
        self.workspace_marker.scale.x, self.workspace_marker.scale.y, self.workspace_marker.scale.z = ws_scale
        self.workspace_marker.color.g = self.workspace_marker.color.a = .5
    
    def get_absolute_mode_flag(self):
        return rospy.get_param(self.absolute_teleop_mode_rosparam, default=True)
    
    def set_absolute_mode_flag(self, value):
        if value==True:
            rospy.loginfo('{}: Receiving targets in world frame!'.format(rospy.get_name()))
        else:
            rospy.loginfo('{}: Receiving targets in in retation to the initial position!'.format(rospy.get_name()))
        rospy.set_param(self.absolute_teleop_mode_rosparam, value)
    
    def goto_pose(self, goal_pose):
        """sends the target point

        Args:
            goal_pose (numpy.array): array containing the position and orientation relative to the initial position
            of the goal.
        """
        # posegoal = EEPoseGoals()
        self.posegoal.header.stamp = rospy.Time.now()
        self.posegoal.ee_poses[0] = pm.toMsg(goal_pose)
        self.goal_pub.publish(self.posegoal)
        
            
    def send_marker(self, pose, frame='world'):
        self.target_marker.pose = pose
        tgt = PoseStamped(pose=pose)
        tgt.header.frame_id = frame
        self.marker_target_pub.publish(tgt)
        self.marker_workspace_pub.publish(self.workspace_marker)
        
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
        diff = (
            np.asarray(joint_angle_msg.angles.data) - \
                np.asarray(self.moveit_interface.get_current_joint_values())).round(3)
        if np.linalg.norm(diff) > 1e-2:
            rospy.loginfo(str(diff))
            self.add_to_buffer(joint_angle_msg.angles.data)
        
    def add_to_buffer(self, js_postion):
        old_jangles = self.jangles
        self.jangles = JointState(name=self.jnames, position=js_postion)
        # diff = (
        #     np.asarray(js_postion) - \
        #         np.asarray(self.moveit_interface.get_current_joint_values())).round(3)
        if self.sim:
            self.jstate_buffer.append(deepcopy(self.jangles))
        else:
            # if np.any(diff > 1e-2):
            n = len(self.jstate_buffer.trajectory.points)
            self.jstate_buffer.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=js_postion,
                    velocities=[np.pi/100]*6, 
                    accelerations=[np.pi/150]*6, 
                    time_from_start=rospy.Duration((n + 1) * self.time_to_target)))
                # self.jstate_buffer.trajectory.points = []
                # self.jstate_buffer.trajectory.points.append(
                #     JointTrajectoryPoint(
                #         positions=js_postion,
                #         velocities=[np.pi/100]*6, 
                #         accelerations=[np.pi/100]*6, 
                #         time_from_start=rospy.Duration(self.time_to_target)))
                # self.debug_buffer_size_pub.publish(Int32(n+1))
            
            
    def consume_buffer(self):
        # jnt_pts = self.make_joint_trajectory(joint_angle_msg.angles.data, 3)
        if self.sim and len(self.jstate_buffer) > 0:
            
            buffer, self.jstate_buffer = deepcopy(self.jstate_buffer), []
            for jstate in buffer:
                # for k, jnts in enumerate(jnt_pts[-1:]):
                # self.jangles.position = joint_angle_msg.angles.data
                jstate.header.stamp = rospy.Time.now()
                # self.jangles.header.seq = k
                self.joint_target_pub.publish(self.jangles)
                rospy.sleep(self.time_to_target)
            
        
        elif not self.sim and len(self.jstate_buffer.trajectory.points) > 0:
            # self.joint_target_pub.wait_for_result() #
            targets, self.jstate_buffer.trajectory.points = deepcopy(self.jstate_buffer), []
            self.joint_target_pub.send_goal(targets)
            rospy.sleep(self.time_to_target / 2)



        else:
            rospy.sleep(self.time_to_target)
        
    def make_joint_trajectory(self, target, steps=3):
        source = self.moveit_interface.get_current_joint_values()
        tgt = np.asarray(target)
        p = np.linspace(0, 1., steps)
        pts = np.zeros(steps, tgt.shape[0])
        pts = p * tgt + (1-p) * source
        # pts = np.linspace(source, target, steps).tolist()
        return pts.tolist()
        
    def OnPoseRequest(self, pose_stamped):
        request = pm.fromMsg(pose_stamped.pose)
        
        if self.get_absolute_mode_flag():
            bound_p = self.workspace.bind(request.p)
            # print(request.p, '---->>')
            request.p = Vector(*bound_p.tolist())
            # print(request.p, '<<---')
            request = self.pose_to_relative_frame(request)
        
        if self.debug_mode:
            
            self.send_marker(
                pm.toMsg(self.correct_bias(request)), 'hand_root')
        corrected = self.correct_bias(request)
        # corrected.p *= 0.1    
        self.goto_pose(
            corrected
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