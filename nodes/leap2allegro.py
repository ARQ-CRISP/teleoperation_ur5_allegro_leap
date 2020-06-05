#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 15/11/19
'''
from __future__ import division, print_function
# from pynput.keyboard import Key, Controller, Listener, KeyCode
from rospkg.rospack import RosPack
from pickle import load
import numpy as np
import rospy
import tf2_ros
from teleoperation_ur5_allegro_leap import Leap_Teleop_Allegro
from teleoperation_ur5_allegro_leap import EventCatcher


def listoffingers_to_dict(lof): #TODO BAD Solution the file should be refactored possibly in a readable format
    fnames = ['f0', 'f2', 'f3', 'f1']
    fingers = dict()
    for i, finger in enumerate(lof):
        # print(i, fnames[i], finger)
        fingers[fnames[i]] = {'pos': {'x': finger[0][0], 'y': finger[0][1], 'z': finger[0][2]},
                              'rot':  {'x': finger[1][0], 'y': finger[1][1], 'z': finger[1][2], 'w': finger[1][3]}}
    return fingers

def set_calibration_pose_param(poses, pose_param):
    #TODO: Other Bad solution, requires better design
    poses['open_hand'] = poses['relax']
    del poses['relax']

    known_poses = dict([(pp['name'], int(p_index[1])) for p_index, pp in rospy.get_param(pose_param).items()])
    indices = []
    pose_idx = len(known_poses)
    for name, pose in poses.items():
        # print(name)
        if name in known_poses.keys():
            indices.append((name, known_poses[name]))
        else:
            indices.append((name, pose_idx))
            pose_idx += 1
    
    max_idx = get_max_pose_index(pose_param)
    indices = dict(indices)
    # indices = [int(key[1]) if value in poses.keys() else 0 for (key, value) in rospy.get_param(pose_param).items()]

    for i, (key, pose) in enumerate(poses.items()):
        
        rospy.loginfo("{:d}, {:s}, {:s} ".format(i, key, pose_param + "p{}/".format(indices[key]) + "name"))
        rospy.set_param(pose_param + "p{}/".format(indices[key]) + "name", key)
        rospy.set_param(pose_param + "p{}/".format(indices[key]) + "state", pose)
        
def get_max_pose_index(pose_param='allegro_hand_kdl/poses/cartesian_poses'):
    pose_idxs = rospy.get_param(pose_param).keys()
    return max([int(pose_idx[-1]) for pose_idx in pose_idxs])
    # rospy.set_param(pose_param + "p{}/".format(2) + "state", pose)
    

if __name__ == "__main__":

    rospy.init_node('leap2allegro')
    tf_buffer = tf2_ros.Buffer(rospy.Duration(50))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    lefthand_mode = rospy.get_param('~left_hand', False)
    leap_topic = rospy.get_param('~leap_topic', '/leap_motion/leap_filtered')
    package_path = RosPack().get_path('relaxed_leap_teleop')

    calibration_file = package_path + '/config/calibration_allegro_states.pkl'
    with open(calibration_file, 'r') as ff:
        calibration_poses = load(ff)
        calibration_poses = dict([(key, listoffingers_to_dict(finger)) for key, finger in calibration_poses.items()])
    # keyboard = Controller()


    set_calibration_pose_param(calibration_poses, Leap_Teleop_Allegro.pose_param)
    allegro_teleop = Leap_Teleop_Allegro(tf_buffer, leap_topic, lefthand_mode, scale = [1.0, 1.0, 1.0, 1.0])
    rospy.on_shutdown(allegro_teleop.on_shutdown)
    stop_calibration = False
    
    
    ec = EventCatcher()
    ec.set_status('Execution_Mode')
    ec.set_title('Teleop Controller')
    # ec.title('Teleop Controller')
    def enter_calibration_mode():
        global stop_calibration
        global ec
        stop_calibration = False
        rospy.loginfo('starting calibration mode...')
        ec.set_status('Calibration_Mode')

    def exit_calibration_mode():
        global stop_calibration
        global ec
        stop_calibration = True
        rospy.loginfo('ending calibration mode...')
        ec.set_status('Execution_Mode')
    
    def is_calibrating():
        global stop_calibration
        return not stop_calibration

    def calibrate_procedure(teleop_obj, pose_name, Finger1, Finger2, eps=1e-3):
            
            if not allegro_teleop.is_calibrating:
                teleop_obj.toggle_calibration_mode()
            rospy.loginfo('Allegro Teleop >> going to calibration mode!')
            teleop_obj.goto_pose_by_name(pose_name)
            enter_calibration_mode()
            # while True:
            do_calibrate(teleop_obj, pose_name, Finger1, Finger2, eps)
            
    def do_calibrate(teleop_obj, pose_name, Finger1, Finger2, eps=1e-3):
        global stop_calibration
        global ec
        fing1 = teleop_obj.leap_hand_tracker.fingers[Finger1].position[-1, :]
        fing2 = teleop_obj.leap_hand_tracker.fingers[Finger2].position[-1, :]
        dist = np.linalg.norm(fing1 - fing2)
        print(chr(27)+'[2j')
        print('\033c')
        print('\x1bc')
        rospy.loginfo('Allegro Teleop >> Make {} and {} fingers touch!'.format(Finger1, Finger2))
        rospy.loginfo('Allegro Teleop >> current distance {:.2f}'.format(dist))
        # print(stop_calibration)
        rospy.sleep(0.2)

        if dist <= eps or stop_calibration:
            if not stop_calibration:
                rospy.loginfo('Allegro Teleop >> Calibration succeded! -> press space')
            else:
                rospy.logwarn('Allegro Teleop >> Calibration terminated prematurely dist: {} -> press space'.format(dist))

            exit_calibration_mode()
            teleop_obj.allegro_state[Finger1].update_measure()
            teleop_obj.allegro_state[Finger2].update_measure()

        else:
            ec.after(1000, lambda: do_calibrate(teleop_obj, pose_name, Finger1, Finger2, eps))
                
    ec.bind_action('Escape', (None, allegro_teleop.toggle_tracking))
    ec.bind_action('space', (None, allegro_teleop.toggle_calibration_mode))
    ec.bind_action('BackSpace', (None, exit_calibration_mode))
    ec.bind_action('F1', (lambda : calibrate_procedure(allegro_teleop, 'index', 'Index', 'Thumb', eps=1e-2), None))
    ec.bind_action('F2', (lambda : calibrate_procedure(allegro_teleop, 'middle', 'Middle', 'Thumb', eps=1e-2), None))
    ec.bind_action('F3', (lambda : calibrate_procedure(allegro_teleop, 'ring', 'Ring', 'Thumb', eps=1.1e-2), None))
    ec.show_keybinders()

    ec.mainloop()
