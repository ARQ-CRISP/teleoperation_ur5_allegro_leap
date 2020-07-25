#! /usr/bin/env python
import rospy
import numpy as np
from rospkg import RosPack
from key_event_catcher import EventCatcher
from teleoperation_ur5_allegro_leap.teleop.allegro.calibration import listoffingers_to_dict, set_calibration_pose_param, get_max_pose_index


class Calibration_GUI():

    def __init__(self, allegro_teleop_interface, event_catcher=None):
        """Calibration gui - Gui containing binders for the calibration of the robot fingers.

        Args:
            allegro_teleop_interface (Leap_Teleop_Allegro): interface to the finger controls
            event_catcher (EventCatcher, optional): Generic Event Catcher GUI Component. Defaults to None.
        """

        self.stop_calibration = False
        self.event_catcher = EventCatcher() if event_catcher is None else event_catcher
        self.allegro_teleop_interface = allegro_teleop_interface
        self.event_catcher.add_frame('Calibration')

        self.event_catcher.menus['File'].add_command(
            label='Start calibration world', command=self.goto_calibration_world)

        self.event_catcher.bind_action('Escape', (None, 
            self.allegro_teleop_interface.toggle_tracking), 'Toggle Tracking mode',frame_name='Calibration')
        
        self.event_catcher.bind_action('space', (None, 
            self.allegro_teleop_interface.toggle_calibration_mode), 'Toggle Calibration mode', frame_name='Calibration')
        
        self.event_catcher.bind_action('BackSpace', (None, 
            self.exit_calibration_mode), 'Exit Calibration mode', frame_name='Calibration')
        
        self.event_catcher.bind_action('F1', (lambda: self.calibrate_procedure(
            'index', 'Index', 'Thumb', eps=1e-2), None), 'Calibrate Index', frame_name='Calibration')
        
        self.event_catcher.bind_action('F2', (lambda: self.calibrate_procedure(
            'middle', 'Middle', 'Thumb', eps=1e-2), None), 'Calibrate Middle', frame_name='Calibration')
        
        self.event_catcher.bind_action('F3', (lambda: self.calibrate_procedure(
            'ring', 'Ring', 'Thumb', eps=1.1e-2), None), 'Calibrate Ring', frame_name='Calibration')


    def enter_calibration_mode(self):

        self.stop_calibration = False
        rospy.loginfo('starting calibration mode...')
        self.event_catcher.set_status('Calibration_Mode')

    def exit_calibration_mode(self):

        self.stop_calibration = True
        rospy.loginfo('ending calibration mode...')
        self.event_catcher.set_status('Execution_Mode')

    def is_calibrating(self):
        return not self.stop_calibration

    def goto_calibration_world(self):
        # try:
        #     if self.event_catcher.top_level is not None:
        #         self.event_catcher.top_level.quit()
        # except e:
        #     pass
        mujpath = RosPack().get_path('allegro_mujoco')
        self.event_catcher.load_simulation(mujpath + '/config/worlds/' + 'calibration-world.xml')

    def calibrate_procedure(self,  pose_name, Finger1, Finger2, eps=1e-3):

        if not self.allegro_teleop_interface.is_calibrating:
            self.allegro_teleop_interface.toggle_calibration_mode()
        rospy.loginfo('Allegro Teleop >> going to calibration mode!')
        self.allegro_teleop_interface.goto_pose_by_name(pose_name)
        self.enter_calibration_mode()
        # while True:
        self.do_calibrate(pose_name, Finger1, Finger2, eps)

    def do_calibrate(self, pose_name, Finger1, Finger2, eps=1e-3):

        fing1 = self.allegro_teleop_interface.leap_hand_tracker.fingers[Finger1].position[-1, :]
        fing2 = self.allegro_teleop_interface.leap_hand_tracker.fingers[Finger2].position[-1, :]
        dist = np.linalg.norm(fing1 - fing2)
        print(chr(27)+'[2j')
        print('\033c')
        print('\x1bc')
        rospy.loginfo(
            'Allegro Teleop >> Make {} and {} fingers touch!'.format(Finger1, Finger2))
        rospy.loginfo('Allegro Teleop >> current distance {:.2f}'.format(dist))
        # print(stop_calibration)
        rospy.sleep(0.2)

        if dist <= eps or self.stop_calibration:
            if not self.stop_calibration:
                rospy.loginfo(
                    'Allegro Teleop >> Calibration succeded! -> press space')
            else:
                rospy.logwarn(
                    'Allegro Teleop >> Calibration terminated prematurely dist: {} -> press space'.format(dist))

            self.exit_calibration_mode()
            self.allegro_teleop_interface.allegro_state[Finger1].update_measure()
            self.allegro_teleop_interface.allegro_state[Finger2].update_measure()

        else:
            self.event_catcher.after(500, lambda: self.do_calibrate(pose_name, Finger1, Finger2, eps))
