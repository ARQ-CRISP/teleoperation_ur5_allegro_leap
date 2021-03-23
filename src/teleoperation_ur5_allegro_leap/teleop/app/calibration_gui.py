#! /usr/bin/env python
import rospy
import numpy as np
from rospkg import RosPack
from key_event_catcher import EventCatcher
from teleoperation_ur5_allegro_leap.teleop.allegro.calibration import listoffingers_to_dict, set_calibration_pose_param, get_max_pose_index
from teleoperation_ur5_allegro_leap.srv import Toggle_Calibration, Toggle_Tracking, GoTo_ByName, Get_Fingertip_Distance, Update_Finger_Measure
tracking_toggler_service = 'allegro_teleop/toggle_tracking'
calibration_toggler_service = 'allegro_teleop/toggle_calibration'
goto_pose_byname_service = 'allegro_teleop/pose_by_name'
fingertip_update_service = 'allegro_teleop/fingertip_update'
fingertip_distance_service = 'allegro_teleop/fingertip_distance'

class Calibration_GUI():

    def __init__(self
                #  , allegro_teleop_interface
                 , event_catcher=None):
        """Calibration gui - Gui containing binders for the calibration of the robot fingers.

        Args:
            allegro_teleop_interface (Leap_Teleop_Allegro): interface to the finger controls
            event_catcher (EventCatcher, optional): Generic Event Catcher GUI Component. Defaults to None.
        """

        self.stop_calibration = False
        self.event_catcher = EventCatcher() if event_catcher is None else event_catcher
        # self.allegro_teleop_interface = allegro_teleop_interface
        self.event_catcher.add_frame('Calibration')
        
        rospy.wait_for_service(tracking_toggler_service)
        rospy.wait_for_service(calibration_toggler_service)
        rospy.wait_for_service(goto_pose_byname_service)
        rospy.wait_for_service(fingertip_distance_service)
        rospy.wait_for_service(fingertip_update_service)
        
        self.__toggle_tracking = rospy.ServiceProxy(tracking_toggler_service, Toggle_Tracking)
        self.__toggle_calibration_mode = rospy.ServiceProxy(calibration_toggler_service, Toggle_Calibration)
        self.__goto_pose_byname = rospy.ServiceProxy(goto_pose_byname_service, GoTo_ByName)
        self.__fingertip_distance = rospy.ServiceProxy(fingertip_distance_service, Get_Fingertip_Distance)
        self.__fingertip_update = rospy.ServiceProxy(fingertip_update_service, Update_Finger_Measure)
        
        self.event_catcher.menus['File'].add_command(
            label='Start calibration world', command=self.goto_calibration_world)

        self.event_catcher.bind_action('Escape', (None, 
            self.toggle_tracking), 'Toggle Tracking mode',frame_name='Calibration')
        
        self.event_catcher.bind_action('space', (None, 
            self.toggle_calibration_mode), 'Toggle Calibration mode', frame_name='Calibration')
        
        self.event_catcher.bind_action('BackSpace', (None, 
            self.exit_calibration_mode), 'Exit Calibration mode', frame_name='Calibration')
        
        # self.event_catcher.bind_action('F1', (lambda: self.calibrate_procedure(
        #     'index', 'Index', 'Thumb', eps=0.011), None), 'Calibrate Index', frame_name='Calibration')
        
        # self.event_catcher.bind_action('F2', (lambda: self.calibrate_procedure(
        #     'middle', 'Middle', 'Thumb', eps=0.011), None), 'Calibrate Middle', frame_name='Calibration')
        
        # self.event_catcher.bind_action('F3', (lambda: self.calibrate_procedure(
        #     'ring', 'Ring', 'Thumb', eps=0.01), None), 'Calibrate Ring', frame_name='Calibration')

        self.event_catcher.bind_action('F6', (lambda: self.goto_open_hand()
            , None), 'Open Hand', frame_name='Calibration')

        self.event_catcher.bind_action('F7', (lambda: self.update_hand_state()
            , None), 'Update Measure', frame_name='Calibration')

    def toggle_tracking(self):
        return self.__toggle_tracking(True)
    
    def is_tracking(self):
        return self.__toggle_tracking(False)
    
    def toggle_calibration_mode(self):
        return self.__toggle_calibration_mode(True)
    
    def is_calibrating(self):
        return self.__toggle_calibration_mode(False)
        
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
        
        mujpath = RosPack().get_path('allegro_mujoco')
        self.event_catcher.load_simulation(mujpath + '/config/worlds/' + 'calibration-world.xml')

    def calibrate_procedure(self, pose_name, Finger1, Finger2, eps=1e-3):

        if not self.is_calibrating():
            self.toggle_calibration_mode()
        rospy.loginfo('Allegro Teleop >> going to calibration mode!')
        
        self.__goto_pose_byname(pose_name)
        self.enter_calibration_mode()
        # while True:
        self.do_calibrate(pose_name, Finger1, Finger2, eps)

    def do_calibrate(self, pose_name, Finger1, Finger2, eps=1e-3):

        # fing1 = self.allegro_teleop_interface.leap_hand_tracker.fingers[Finger1].position[-1, :]
        # fing2 = self.allegro_teleop_interface.leap_hand_tracker.fingers[Finger2].position[-1, :]
        dist = round(self.__fingertip_distance(Finger1, Finger2), 3)
        # dist = round(np.linalg.norm(fing1 - fing2), 3)
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
            self.__fingertip_update(Finger1)
            self.__fingertip_update(Finger2)
            # self.allegro_teleop_interface.allegro_state[Finger1].update_measure()
            # self.allegro_teleop_interface.allegro_state[Finger2].update_measure()

        else:
            self.event_catcher.after(500, lambda: self.do_calibrate(pose_name, Finger1, Finger2, eps))

    def goto_open_hand(self):
        if not self.is_calibrating():
            self.toggle_calibration_mode()
        rospy.loginfo('Allegro Teleop >> going to calibration mode!')
        self.enter_calibration_mode()
        # self.allegro_teleop_interface.goto_pose_by_name('wide_open')
        # self.allegro_teleop_interface.goto_pose_by_name('relax')
        self.__goto_pose_byname('relax')

    def update_hand_state(self):
        if not self.is_calibrating():
            self.toggle_calibration_mode()
        rospy.loginfo('Allegro Teleop >> going to calibration mode!')
        self.enter_calibration_mode()
        # self.allegro_teleop_interface.goto_pose_by_name('wide_open')
        for finger in ['Thumb', 'Index', 'Middle', 'Ring']:
            self.__fingertip_update(finger)
            # self.allegro_teleop_interface.allegro_state[finger].update_measure()

