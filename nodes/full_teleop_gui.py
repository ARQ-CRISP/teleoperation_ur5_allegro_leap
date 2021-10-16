#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 01/08/21
'''
from __future__ import print_function, absolute_import, division
from rospy.core import rospyinfo
import tf2_py
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import PoseStamped
import PyKDL as kdl
import tf2_ros
from tf_conversions import posemath as pm

from teleoperation_ur5_allegro_leap import EventCatcher#, Calibration_GUI, Movegroup_GUI, Experiments_GUI
from teleoperation_ur5_allegro_leap.srv import Arm_Cartesian_Target
from teleoperation_ur5_allegro_leap.srv import Toggle_Tracking, Toggle_TrackingResponse
from teleoperation_ur5_allegro_leap.srv import Toggle_ArmTeleopMode, Toggle_ArmTeleopModeResponse
from teleoperation_ur5_allegro_leap.srv import GoTo_ByName
from teleoperation_ur5_allegro_leap.srv import Go_To_Base
from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix
import Tkinter as tk
import rospy

allegro_telop_prefx = 'allegro_teleop/'

class Teleop_GUI():
    delta_entry_name = 'delta'
    
    marker_request_topic = ur5_teleop_prefix + 'move_marker_pose'
    toggle_teleop_mode_srv = ur5_teleop_prefix + 'toggle_teleop_mode'
    toggle_tracking_arm_srv = ur5_teleop_prefix + 'toggle_tracking'
    toggle_tracking_hand_srv = allegro_telop_prefx + 'toggle_tracking'
    hand_pose_byname_srv = allegro_telop_prefx + 'pose_by_name'
    toggle_finger_hold_srv = allegro_telop_prefx + 'toggle_finger_lock' 
    go_to_base_srv = ur5_teleop_prefix + 'go_to_base'
    toggle_hand_orientation_lock_srv = ur5_teleop_prefix + 'toggle_rotation_lock'
    
    
    def __init__(self, step=0.01, width=350, height=650, arm=True, hand=True):
        
        self.tracking = {'arm': True, 'hand': True}
        self.tracking_mode = {'arm': '', 'hand': 'Active'}
        self.step = step
        self.current_pose = kdl.Frame
        self.buffer = Buffer(rospy.Duration(1))
        self.__listener = TransformListener(self.buffer)
        self.master = tk.Tk()
        self.event_catcher = EventCatcher(self.master, size=(width, height))
        self.hand_state_frame = tk.Frame(self.event_catcher.frames['status_frame'])
        self.hand_state_frame.pack(fill=tk.BOTH, padx=5, pady=5, side=tk.TOP)

        self.hand_status_string = tk.StringVar(self.hand_state_frame, '')
        self.hand_status_label = tk.Label(self.hand_state_frame, textvariable=self.hand_status_string, anchor='center', 
                        font=("Helvetica", 10),
                        borderwidth=5).pack(fill=tk.X, expand=True, padx=5, pady=5)

        self.event_catcher.create_entry(self.delta_entry_name, 'Movement Delta', default_value=str(step))
        self.set_services(arm, hand)
        self.set_commands(arm, hand)
        rospy.Timer(rospy.Duration(1/10), lambda msg: self.listen_ee() )
        
    def start(self):
        self.event_catcher.mainloop()
    
    def goto_hand_pose(self, name='relax'):
        rospy.loginfo('Sending Hand to {} pose'.format(name))
        self.tracking['hand'] = self.__toggle_hand_tracking(update=False).is_tracking
        if self.tracking['hand']:
            self.toggle_tracking(part='hand')
        self.__go_to_hand(name)
    
    def set_services(self, arm=True, hand=True):
        
        if arm:
            rospy.wait_for_service(self.marker_request_topic)
            rospy.wait_for_service(self.toggle_tracking_arm_srv)
            rospy.wait_for_service(self.toggle_teleop_mode_srv)
            rospy.wait_for_service(self.go_to_base_srv)
            rospy.wait_for_service(self.toggle_hand_orientation_lock_srv)
            self.__go_to_arm_base = rospy.ServiceProxy(self.go_to_base_srv, Go_To_Base)
            self.move_marker_by = rospy.ServiceProxy(self.marker_request_topic, Arm_Cartesian_Target)
            self.__toggle_arm_tracking = rospy.ServiceProxy(self.toggle_tracking_arm_srv, Toggle_Tracking)
            self.tracking['arm'] = self.__toggle_arm_tracking(update=False).is_tracking
            self.__toggle_teleop_mode = rospy.ServiceProxy(self.toggle_teleop_mode_srv, Toggle_ArmTeleopMode)
            self.tracking_mode['arm'] = self.__toggle_teleop_mode(update=False).mode
            self.__toggle_arm_orientation = rospy.ServiceProxy(self.toggle_hand_orientation_lock_srv, Toggle_Tracking)
            self.arm_orientation_lock = self.__toggle_arm_orientation(update=False).is_tracking
            
        if hand:
            rospy.wait_for_service(self.toggle_tracking_hand_srv)
            rospy.wait_for_service(self.hand_pose_byname_srv)
            rospy.wait_for_service(self.toggle_finger_hold_srv)
            self.__toggle_hand_tracking = rospy.ServiceProxy(self.toggle_tracking_hand_srv, Toggle_Tracking)
            self.__go_to_hand = rospy.ServiceProxy(self.hand_pose_byname_srv, GoTo_ByName)
            self.tracking['hand'] = self.__toggle_hand_tracking(update=False).is_tracking
            self.__toggle_finger_lock = rospy.ServiceProxy(self.toggle_finger_hold_srv, Toggle_Tracking)
            self.finger_lock = self.__toggle_finger_lock(update=False).is_tracking

    def go_to_arm_base(self):
        if self.tracking['arm']:
            self.toggle_tracking('arm')
        
        self.__go_to_arm_base()
        self.master.after(500, lambda : self.toggle_tracking('arm') if not self.tracking['arm'] else False)
        # if not self.tracking['arm']:
        #     self.__toggle_teleop_mode(True)

    def update_status_string(self):
        arm_status = 'Stop' if not self.tracking['arm'] else self.tracking_mode['arm']
        hand_status = 'Stop' if not self.tracking['hand'] else self.tracking_mode['hand']
        status = '\n'.join([
            'Arm Teleop: {}'.format(arm_status), 
            'Hand Teleop: {}'.format(hand_status),
            'Hand Orientation Locked: {}'.format(self.arm_orientation_lock),
            'Finger Locked: {}'.format(self.finger_lock),
            ])
        self.event_catcher.status_string.set(status)
    
    def set_commands(self, arm=True, hand=True):
        self.event_catcher.set_title("Teleop Controller")
        if arm:
            self.event_catcher.add_frame('ur_control')
            self.event_catcher.add_frame('arm_modes')
            move_str = 'Move Marker along {}'
            self.event_catcher.bind_action('s',
                (lambda: self.translate([self.get_delta(), 0, 0]),  None), move_str.format('-x'), 'ur_control')
            self.event_catcher.bind_action('w',
                (lambda: self.translate([-self.get_delta(), 0, 0]), None), move_str.format('+x'), 'ur_control')
            self.event_catcher.bind_action('d',
                (lambda: self.translate([0, self.get_delta(), 0]),  None), move_str.format('+y'), 'ur_control')
            self.event_catcher.bind_action('a',
                (lambda: self.translate([0, -self.get_delta(), 0]), None), move_str.format('-y'), 'ur_control')
            self.event_catcher.bind_action('e',
                (lambda: self.translate([0, 0, self.get_delta()]),  None), move_str.format('+z'), 'ur_control')
            self.event_catcher.bind_action('q',
                (lambda: self.translate([0, 0, -self.get_delta()]), None), move_str.format('-z'), 'ur_control')
            
            self.event_catcher.bind_action('F1', (
                lambda: self.toggle_tracking(part='arm'),
                None), 'Toggle Arm Tracking' , 'arm_modes')
            
            self.event_catcher.bind_action('F3', (
                lambda: self.toggle_tracking_mode('arm'),
                None), 'Toggle Arm Tracking Mode' , 'arm_modes')
            
            self.event_catcher.bind_action('F4', (
                lambda: self.toggle_arm_orientation_lock(),
                None), 'Toggle Hand Orientation Lock' , 'arm_modes')
            
            self.event_catcher.bind_action('F8', (
                lambda: self.go_to_arm_base(),
                None), 'Go to Arm Base Pose' , 'arm_modes')

        
        if hand:
            self.event_catcher.add_frame('hand_modes')
            self.event_catcher.bind_action('F2', (
                lambda: self.toggle_tracking(part='hand'),
                None), 'Toggle Hand Tracking' , 'hand_modes')
            
            self.event_catcher.bind_action('F5', (
                lambda: self.toggle_finger_lock(),
                None), 'Toggle Hand Tracking' , 'hand_modes')
            
            self.event_catcher.bind_action('o', (
                lambda: self.goto_hand_pose('open_hand'),
                None), 'Go to Open Hand' , 'hand_modes')
            
            self.event_catcher.bind_action('p', (
                lambda: self.goto_hand_pose('wide_open'),
                None), 'Go to Wide Open' , 'hand_modes')
            
            self.event_catcher.bind_action('i', (
                lambda: self.goto_hand_pose('relax'),
                None), 'Go to Relax' , 'hand_modes')
        
        self.update_status_string()
        self.event_catcher.show_keybinders()

    def toggle_tracking(self, part='arm'):
        if part == 'arm':
            self.tracking[part] = self.__toggle_arm_tracking(update=True).is_tracking
        elif part == 'hand':
            self.tracking[part] = self.__toggle_hand_tracking(update=True).is_tracking
        rospy.loginfo('Toggle tracking of the robot {}: {}'.format(part, self.tracking[part]))
        self.update_status_string()
        
    def toggle_tracking_mode(self, part='arm'):
        self.tracking_mode[part] = self.__toggle_teleop_mode(update=True).mode
        rospy.loginfo('Tracking mode of the robot {}: {}'.format(part, self.tracking_mode[part]))
        self.update_status_string()
        
    def toggle_arm_orientation_lock(self):
        self.arm_orientation_lock = self.__toggle_arm_orientation(update=True).is_tracking
        rospy.loginfo('Hand Orientation Locked: {}'.format(self.arm_orientation_lock))
        self.update_status_string()
        
    def toggle_finger_lock(self):
        self.finger_lock = self.__toggle_finger_lock(update=True).is_tracking
        rospy.loginfo('Fingers Locked: {}'.format(self.finger_lock))
        self.update_status_string() 

    def get_delta(self):
        return self.event_catcher.get_entry_value(self.delta_entry_name)

    def listen_ee(self):
        try:
            if self.buffer.can_transform('world', 'hand_root', rospy.Time.now(), rospy.Duration(1/10.)):
                transform = self.buffer.lookup_transform('world', 'hand_root', rospy.Time.now(), rospy.Duration(1/10.)).transform
                tras = [transform.translation.x, transform.translation.y, transform.translation.z]
                rot = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
                self.current_pose = pm.fromTf((tras, rot)
                    )
                self.set_hand_position_string()
        except tf2_ros.TransformException as e:
            rospy.logwarn(str(e))
                            
    def set_hand_position_string(self):
        pos_string = 'Position: ({:.3f}, {:.3f}, {:.3f})'.format(*list(self.current_pose.p))
        self.hand_status_string.set(pos_string)
        
    def translate(self, direction):
        target = kdl.Frame()
        target.p = kdl.Vector(*direction)
        
        self.move_marker_by(query=PoseStamped(pose=pm.toMsg(target)), absolute=False)
        # print(target.p)


if __name__ == '__main__':
    
    rospy.init_node('GUI_Node')
    arm_teleop = rospy.get_param('~arm_teleop', True)
    hand_teleop = rospy.get_param('~hand_teleop', True)
    print('arm_teleop = {} hand_teleop = {}'.format(arm_teleop, hand_teleop))
    gui = Teleop_GUI(
        arm=arm_teleop,
        hand=hand_teleop)
    rospy.on_shutdown(gui.master.destroy)
    gui.start()