#! /usr/bin/env python
from __future__ import print_function, division
import numpy as np
import rospy
from std_msgs.msg import String
from teleoperation_ur5_allegro_leap.msg import Experiment
import yaml
from tf2_ros import Buffer, TransformListener
from key_event_catcher import EventCatcher
from timer import RealTimer
from rospkg.rospack import RosPack
from Tkinter import Menu, Listbox, Frame, Button,  BOTH, TOP, LEFT, RIGHT, Toplevel, CENTER, Label, StringVar, RIDGE, X
from Tkconstants import BOTTOM
from moveit_commander.conversions import transform_to_list
import numpy as np
from functools import partial
import subprocess, shlex

BASE_PATH = RosPack().get_path('teleoperation_ur5_allegro_leap') + '/src/teleoperation_ur5_allegro_leap/config/'
TOPICS = [
    "/optoforce_wrench_0", 
    "/optoforce_wrench_1",
    "/optoforce_wrench_2",
    "/optoforce_wrench_3",
    "/allegroHand_0/desired_forces",
    "/allegroHand_0/joint_states",
    "/allegroHand_0/torque_cmd",
    "/leap_motion/leap_device",
    "/leap_motion/leap_filtered",
    "/leap_motion/visualization_marker_array",
    "/allegro_teleop_debug",
    "/tf",
    "/tf_static",
    "/vibration_command",
    "/vibration_state",
    "/experiment_state"
    ]


class Experiment_Panel(Toplevel):
    def __init__(self, master, name, moveit_gui, tf_buffer, seconds_to_closure=5, cnf={}, **kw):
        Toplevel.__init__(self, master=master, cnf=cnf, **kw)
        self.name = name
        self.geometry('300x350')
        self.seconds_to_closure = seconds_to_closure
        self.moveit_gui = moveit_gui
        self.tf_buffer = tf_buffer
        self.title('Experiment Starter: ' + name)
        self.window_frame = Frame(self)
        self.button_frame = Frame(self)
        

        self.grasp_button = Button(self.button_frame, text='Start Grasping', 
            command=self.grasping_experiment, justify=CENTER, padx=5, pady=5)
        self.manip_button = Button(self.button_frame, text='Start Manipulation', 
            command=self.manipulation_experiment, justify=CENTER, padx=5, pady=5)
        self.expall_button = Button(self.button_frame, text='Start Exp-ALL', 
            command=self.experiment_all, justify=CENTER, padx=5, pady=5)
        self.button_frame.pack(fill=BOTH, padx=5, pady=5, side=TOP)

        self.labels_frame = Frame(self.window_frame)
        self.label_frames = dict()
        self.label_frames['grasp'] = Frame(self.labels_frame)
        self.label_frames['hold'] = Frame(self.labels_frame)
        self.label_frames['manipulation'] = Frame(self.labels_frame)

        self.grasp_success_text = StringVar(self.labels_frame, '')
        self.hold_success_text = StringVar(self.labels_frame, '')
        self.manipulation_success_text = StringVar(self.labels_frame, '')

        self.grasp_countdown_text = StringVar(self.labels_frame, str(seconds_to_closure)+'s')
        self.hold_countdown_text = StringVar(self.labels_frame, str(seconds_to_closure)+'s')
        self.manipulation_countdown_text = StringVar(self.labels_frame, str(seconds_to_closure)+'s')
        
        grasp_countdown_label = Label(
            self.label_frames['grasp'], textvariable=self.grasp_countdown_text, relief=RIDGE, width=10, height=2, font=("Helvetica", 15))
        hold_countdown_label = Label(
            self.label_frames['hold'], textvariable=self.hold_countdown_text, relief=RIDGE, width=10, height=2, font=("Helvetica", 15))
        manipulation_countdown_label = Label(
            self.label_frames['manipulation'], textvariable=self.manipulation_countdown_text, relief=RIDGE, width=10, height=2, font=("Helvetica", 15))
        
        grasp_success_label = Label(
            self.label_frames['grasp'], textvariable=self.grasp_success_text, relief=RIDGE, width=15, height=3, font=("Helvetica", 10))
        hold_success_label = Label(
            self.label_frames['hold'], textvariable=self.hold_success_text, relief=RIDGE, width=15, height=3, font=("Helvetica", 10))
        manipulation_success_label = Label(
            self.label_frames['manipulation'], textvariable=self.manipulation_success_text, relief=RIDGE, width=15, height=3, font=("Helvetica", 10))
        
        self.window_frame.pack(fill=BOTH, padx=5, pady=5, side=TOP)
        self.grasp_button.pack(fill=BOTH, padx=5, pady=5, side=LEFT)
        self.manip_button.pack(fill=BOTH, padx=5, pady=5, side=RIGHT)
        self.expall_button.pack(fill=X, padx=5, pady=5, side=BOTTOM)

        self.button_frame.pack(fill=BOTH, padx=5, pady=5, side=TOP)
        self.labels_frame.pack(fill=BOTH, padx=5, pady=5, side=TOP)

        for frame in self.label_frames.values():
            frame.pack(fill=X, padx=2, pady=3, side=TOP)

        grasp_success_label.pack(fill=X, pady=5, side=RIGHT)
        hold_success_label.pack(fill=X, pady=5, side=RIGHT)
        manipulation_success_label.pack(fill=X, pady=5, side=RIGHT)
        grasp_countdown_label.pack(fill=X, pady=5, side=LEFT)
        hold_countdown_label.pack(fill=X, pady=5, side=LEFT)
        manipulation_countdown_label.pack(fill=X, pady=5, side=LEFT)
        self.timers = {'ongoing': None, 'incoming': None}

        self.exp_pub = rospy.Publisher('experiment_state', Experiment, queue_size=100)

        self.success = [False, False, False]
        self.bind("<KeyPress>", master.keydown)
        self.bind("<KeyRelease>", master.keyup)

    def get_object_pose(self):
        tf_stamped = self.tf_buffer.lookup_transform('object', 'world', rospy.Time().now(), rospy.Duration(0.2))
        xyzq = transform_to_list(tf_stamped.transform)
        return np.array(xyzq)

    def get_palm_pose(self):
        tf_stamped = self.tf_buffer.lookup_transform('palm_link', 'world', rospy.Time().now(), rospy.Duration(0.5))
        xyzq = transform_to_list(tf_stamped.transform)
        return np.array(xyzq)
    
    def obj_dist(self, to='palm_link'):
        
        diff = transform_to_list(self.tf_buffer.lookup_transform(
            to, 'object', rospy.Time(0), rospy.Duration(0.3)).transform)
        return np.linalg.norm(diff[0:3], 2)

    def check_grasp(self):
        dist = self.obj_dist()
        print('Grasp Measurement')
        self.success[0] = dist < 0.12
        self.grasp_success_text.set(
            '{:.3f}'.format(dist)# + ' -> ' + str(self.success[0])
            )

    def check_hold(self):
        dist = self.obj_dist()
        print('Hold Measurement')
        self.success[1] = dist < 0.12
        self.hold_success_text.set(
            '{:.3f}'.format(dist)# + ' -> ' + str(self.success[1])
            )
    
    def check_manipulation(self):
        dist = self.obj_dist('target')
        print('Manipulation Measurement')
        self.success[2] = dist > 0.05
        self.manipulation_success_text.set(
            '{:.3f}'.format(dist)# + ' -> ' + str(self.success[2])
            )

    def periodical_callback(self, callback, period_s, duration_s):
        # periodic = rospy.Timer(rospy.Duration(period_s), callback)
        periodic = RealTimer(rospy.Duration(period_s), callback)
        # rospy.Timer(rospy.Duration(duration_s), callback=lambda t: periodic.shutdown(), oneshot=True)
        RealTimer(rospy.Duration(duration_s), callback=lambda t: periodic.shutdown(), oneshot=True)

    def experiment_countdown(self, fun, after_sec, string):
        
        # self.periodical_callback(lambda time: string.set())
        if self.timers['ongoing'] is not None:
            self.timers['ongoing'].shutdown()

        remain = round(min(1.0, after_sec), 3)
        string.set(str((after_sec)) + 's')
        if remain >= 1e-3:
            print('Countdown: {:.3f}'.format(after_sec))
            # self.timers['ongoing'] = rospy.Timer(
            #     rospy.Duration(remain), 
            #     lambda x : self.experiment_countdown(fun, after_sec - remain, string), oneshot=True)
            self.timers['ongoing'] = RealTimer(
                rospy.Duration(remain), 
                lambda x : self.experiment_countdown(fun, after_sec - remain, string), oneshot=True)
        else:
            fun()

    def experiment_all(self):
        self.grasping_experiment()
        if self.timers['incoming'] is not None:
            self.timers['incoming'].shutdown()

        # self.timers['incoming'] = rospy.Timer(
        #     rospy.Duration(
        #         2.2 * self.seconds_to_closure), callback=lambda x: self.manipulation_experiment(), oneshot=True)
        self.timers['incoming'] = RealTimer(
            rospy.Duration(
                2.5 * self.seconds_to_closure), callback=lambda x: self.manipulation_experiment(), oneshot=True)
        # self.after(int(2300 * self.seconds_to_closure), self.manipulation_experiment)

    def grasping_experiment(self):

        def move_arm_and_measure():
            self.moveit_gui.translate([0.0, 0.0, 0.15])
            self.after(500, self.check_grasp)
            self.experiment_countdown(self.check_hold, self.seconds_to_closure, self.hold_countdown_text)

        self.publish_experiment_state('experiment_grasping_' + self.name, self.seconds_to_closure * 2.1)
        self.experiment_countdown(move_arm_and_measure, self.seconds_to_closure, self.grasp_countdown_text)
        # self.after(int(3 * self.seconds_to_closure * 1000), self.manipulation_experiment)

    def manipulation_experiment(self):
        # self.start_recording('experiment_manipulation_' + self.name, self.seconds_to_closure * 1.5)
        # self.after(int(2. * self.seconds_to_closure * 1000), lambda: print('End of Rosbag Recording!'))
        def move_arm_and_measure():
            # self.moveit_gui.translate([0.0, 0.0, -0.15])
            # self.after(500, self.check_manipulation)
            self.check_manipulation()
            
        self.publish_experiment_state('experiment_manipulation_' + self.name, self.seconds_to_closure * 1.3)
        self.moveit_gui.translate([0.0, 0.0, -0.1])
        # self.experiment_countdown(self.check_manipulation, self.seconds_to_closure, self.manipulation_countdown_text)
        self.experiment_countdown(move_arm_and_measure, self.seconds_to_closure, self.manipulation_countdown_text)

    def publish_experiment_state(self, msg_str, secs, begin=True):

        msg = Experiment()
        msg.name = msg_str
        msg.status = '[BEGIN]'
        msg.header.stamp = rospy.Time().now()
        self.exp_pub.publish(msg)

        def send_experiment_msg(timer_event, name_experiment, status = '[ONGOING]'):
            msg = Experiment()
            msg.name = name_experiment
            msg.status = status
            msg.header.stamp = timer_event.rostime
            msg.real_stamp = timer_event.current_expected
            self.exp_pub.publish(msg)
        
        self.periodical_callback(lambda x: send_experiment_msg(x, msg_str, '[ONGOING]'), 1/60., secs)
        # rospy.Timer(rospy.Duration(secs), callback=lambda x: send_experiment_msg(x, msg_str, '[ENDING]'), oneshot=True)
        RealTimer(rospy.Duration(secs), callback=lambda x: send_experiment_msg(x, msg_str, '[ENDING]'), oneshot=True)




class Experiments_GUI():

    def __init__(self, moveit_gui, event_catcher=None, experiments_yaml_name='experiments.yaml'):
        self.top_level = None
        self.moveit_gui = moveit_gui
        self.event_catcher = EventCatcher() if event_catcher is None else event_catcher
        self.tf_buffer = Buffer(rospy.Time(1000))
        self.tf_listener = TransformListener(self.tf_buffer)
        
        self.default_pos = [0.0, 0.0, 0.0]
        self.default_orient = [0.707, 0.0, -0.707, 0.0]

        with open(BASE_PATH + experiments_yaml_name) as ff:
            yaml_exp_data = yaml.load(ff)
            self.experiments_dict = dict([(experiment['name'], experiment) for experiment in yaml_exp_data['experiments']])
            self.seconds_to_closure = yaml_exp_data['experiment_settings']['sec_to_closure']
        

        # self.event_catcher.menubar
        self.event_catcher.define_menu('Load Experiments')
        # self.experiment_menu = Menu(self.event_catcher.menus['base'], tearoff=0)

        for experiment_name, experiment in self.experiments_dict.items():
            # print(experiment_name, experiment)
            self.event_catcher.menus['Load Experiments'].add_command(
                label=experiment_name, command=partial(self.load_experiment, experiment))


        # self.event_catcher.menubar.add_cascade(label="Load Experiments", menu=self.experiment_menu)

    def load_experiment(self, experiment):
        init_pos = experiment['init_position'] if 'init_position' in experiment else self.default_pos
        init_orient = experiment['init_orientation'] if 'init_orientation' in experiment else self.default_orient
        
        self.moveit_gui.goto(init_pos, init_orient)
        self.event_catcher.load_simulation(experiment['worldfile'])
        # bootstrap = [el for el in init]
        # bootstrap[2] += .3
        # self.moveit_gui.goto(bootstrap)
        # rospy.sleep(.5)
        def close_top_level():
            for key, timer in self.top_level.timers.items():
                if timer is not None:
                    timer.shutdown()
            self.top_level.destroy()
            self.top_level = None
            print('Experiment closed!!!')
        if self.top_level is not None and self.top_level:
            close_top_level()
        
        self.top_level = Experiment_Panel(self.event_catcher, experiment['name'], self.moveit_gui, self.tf_buffer, self.seconds_to_closure)
        self.top_level.protocol("WM_DELETE_WINDOW", close_top_level)

