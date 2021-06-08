#! /usr/bin/env python
import rospy
import numpy as np
from key_event_catcher import EventCatcher
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from Tkinter import StringVar, Frame, Label, BOTH, TOP, X

class Movegroup_GUI():

    delta_entry_name = 'delta'

    def __init__(self, arm_group='ur5_arm', event_catcher=None, step=0.01):

        self.movegroup = MoveGroupCommander(arm_group)
        self.event_catcher = EventCatcher() if event_catcher is None else event_catcher
        self.current_pose = self.movegroup.get_current_pose()

        self.hand_state_frame = Frame(self.event_catcher.frames['status_frame'])
        self.hand_state_frame.pack(fill=BOTH, padx=5, pady=5, side=TOP)

        self.hand_status_string = StringVar(self.hand_state_frame, '')
        self.hand_status_label = Label(self.hand_state_frame, textvariable=self.hand_status_string, anchor='center', 
                        font=("Helvetica", 10),
                        borderwidth=5).pack(fill=X, expand=True, padx=5, pady=5)

        self.event_catcher.create_entry(self.delta_entry_name, 'Movement Delta', default_value=str(step))
        self.step = step
        # self.event_catcher.get_entry(self.delta_entry_name)..bind('Enter')
        self.event_catcher.add_frame('ur_control')
        self.event_catcher.bind_action('s', (lambda: self.translate([self.get_delta(), 0, 0]),  None), 'Move UR5 EE along -x'  , 'ur_control')
        self.event_catcher.bind_action('w', (lambda: self.translate([-self.get_delta(), 0, 0]), None), 'Move UR5 EE along +x' , 'ur_control')
        self.event_catcher.bind_action('d', (lambda: self.translate([0, self.get_delta(), 0]),  None), 'Move UR5 EE along +y'  , 'ur_control')
        self.event_catcher.bind_action('a', (lambda: self.translate([0, -self.get_delta(), 0]), None), 'Move UR5 EE along -y' , 'ur_control')
        self.event_catcher.bind_action('e', (lambda: self.translate([0, 0, self.get_delta()]),  None), 'Move UR5 EE along +z'  , 'ur_control')
        self.event_catcher.bind_action('q', (lambda: self.translate([0, 0, -self.get_delta()]), None), 'Move UR5 EE along -z' , 'ur_control')

        self.goto([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
            ], [0.707, 0.0, -0.707, 0.0])

    def translate(self, xyz):
        self.current_pose.pose.position.x += xyz[0]
        self.current_pose.pose.position.y += xyz[1]
        self.current_pose.pose.position.z += xyz[2]

        # self.movegroup.set_pose_target(self.current_pose)
        self.movegroup.set_joint_value_target(self.current_pose, 'hand_root', False)
        plan = self.movegroup.plan()
        self.movegroup.execute(plan)
        self.set_hand_position_string()

    def goto(self, xyz, q=None):

        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = 'world'
        stamped_pose.header.stamp = rospy.Time().now()
        stamped_pose.pose.position.x = xyz[0]
        stamped_pose.pose.position.y = xyz[1]
        stamped_pose.pose.position.z = xyz[2]
        if q is None:
            stamped_pose.pose.orientation = self.current_pose.pose.orientation
        else:
            stamped_pose.pose.orientation.x = q[0]
            stamped_pose.pose.orientation.y = q[1]
            stamped_pose.pose.orientation.z = q[2]
            stamped_pose.pose.orientation.w = q[3]

        self.movegroup.set_joint_value_target(stamped_pose, 'hand_root', True)
        plan = self.movegroup.plan()
        self.movegroup.execute(plan)
        self.current_pose = self.movegroup.get_current_pose(end_effector_link='hand_root')
        self.set_hand_position_string()

    def get_delta(self):
        return self.event_catcher.get_entry_value(self.delta_entry_name)

    def set_hand_position_string(self):
        pos_string = 'Position: ({:.3f}, {:.3f}, {:.3f})'.format(
            self.current_pose.pose.position.x, 
            self.current_pose.pose.position.y, 
            self.current_pose.pose.position.z
            )
        self.hand_status_string.set(pos_string)
        
