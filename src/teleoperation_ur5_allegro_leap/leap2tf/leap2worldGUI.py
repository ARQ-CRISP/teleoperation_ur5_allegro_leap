#! /usr/bin/env python
'''
author: Claudio Coppola
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/10/18
'''
######################################################################################################

from __future__ import print_function
# from relaxed_leap_teleop.config import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
#     joint_state_define, collision_file_name, fixed_frame, config_file_name
# from RelaxedIK.relaxedIK import RelaxedIK
# from relaxed_ik.msg import EEPoseGoals

import os
import math
import yaml
from copy import deepcopy
from collections import OrderedDict

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
import rospy
# import roslaunch
import rospkg
import tf
# from abc import ABCMeta, abstractmethod
# from leap_motion.msg import leapros, Human, Hand, Finger, Bone

import tkinter as tk

# tf.transformations.quaternion_from_euler(math.pi, 0, math.pi),
tf_pub = tf.TransformBroadcaster()
def pose2str(pose):
    rpy = tf.transformations.euler_from_quaternion([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z], 'sxyz')
    pos_str = ['position'] + map(lambda x: str(round(x,3)),[pose.position.x, pose.position.y, pose.position.z])
    orient_str = ['orientation'] + map(lambda x: str(round(x*180.0/math.pi,3)), rpy)
    return ' '.join(pos_str + ['\n'] + orient_str)
    
def static_transform(rpy, source='world', target='leap_hands'):
    # tf_pub.sendTransform((0, 0, 0),
    #                     tf.transformations.quaternion_from_euler(0, 0, 0),
    #                     rospy.Time.now(),
    #                     'common_world',
    #                     fixed_frame)
    
    tf_pub.sendTransform((0, 0, 0),
                        tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2],'sxyz'),
                        rospy.Time.now(),
                        target,
                        source,
                        )



class GUI(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master, width=1080, height=720)
        self.master = master
        self.pack(fill=tk.BOTH)
        self.create_widgets()
        self.rpy = [0., 0., 0.]

    def create_widgets(self):
        top_frame = tk.Frame(self)
        top_frame.pack(side=tk.TOP, fill=tk.X, pady=5)

        source_name_frame = tk.Frame(top_frame)
        source_name_frame.pack(side=tk.LEFT, fill=tk.X)

        target_name_frame = tk.Frame(top_frame)
        target_name_frame.pack(side=tk.RIGHT, fill=tk.X)

        separator_frame = tk.Frame(top_frame, height=10, bd=1)
        separator_frame.pack(fill=tk.NONE, padx=5, pady=5)

        source = tk.StringVar(self, value='world')
        target = tk.StringVar(self, value='leap_hands')

        self.source_label = tk.Label(source_name_frame, text="source", fg="black")
        self.source_label.pack(side=tk.LEFT, padx=5, pady=5)
        self.target_label = tk.Label(target_name_frame, text="target", fg="black")
        self.target_label.pack(side=tk.LEFT, padx=5, pady=5)

        self.source = tk.Entry(source_name_frame, textvariable=source)#, width=10)
        self.source.pack(side=tk.LEFT,fill=tk.NONE, expand=tk.FALSE)#, width=10)
        self.target = tk.Entry(target_name_frame, textvariable=target)
        self.target.pack(side=tk.LEFT, fill=tk.NONE, expand=tk.FALSE)

        vspace = tk.Frame(self, height=2, bd=1)
        vspace.pack(fill=tk.X, padx=5, pady=5)

        slider_separator = tk.Frame(self, height=2, bd=1, relief=tk.SUNKEN)
        slider_separator.pack(fill=tk.X, padx=5, pady=5)

        sliders_frame = tk.Frame(self)
        sliders_frame.pack(side=tk.BOTTOM, fill=tk.BOTH)

        slider_frame_Y = tk.Frame(sliders_frame)
        slider_frame_Y.pack(fill=tk.X)
        slider_frame_P = tk.Frame(sliders_frame)
        slider_frame_P.pack(fill=tk.X)
        slider_frame_R = tk.Frame(sliders_frame)
        slider_frame_R.pack(fill=tk.X)

        self.Y_label = tk.Label(slider_frame_Y, text="Y", fg="black")
        self.Y_label.pack(side=tk.LEFT, fill=tk.NONE, padx=10)
        self.slider_Y = tk.Scale(slider_frame_Y, from_=-180, to=180, tickinterval=90, length=300, orient=tk.HORIZONTAL, resolution=45)
        self.slider_Y.pack(side=tk.RIGHT, fill=tk.X, expand=tk.TRUE, padx=30)

        self.P_label = tk.Label(slider_frame_P, text="P", fg="black")
        self.P_label.pack(side=tk.LEFT, fill=tk.X, padx=10)
        self.slider_P = tk.Scale(slider_frame_P, from_=-180, to=180, tickinterval=90, length=300, orient=tk.HORIZONTAL, resolution=45)
        self.slider_P.pack(side=tk.RIGHT, fill=tk.X, expand=tk.TRUE, padx=30)

        self.R_label = tk.Label(slider_frame_R, text="R", fg="black")
        self.R_label.pack(side=tk.LEFT, fill=tk.X, padx=10)
        self.slider_R = tk.Scale(slider_frame_R, from_=-180, to=180, tickinterval=90, length=300, orient=tk.HORIZONTAL, resolution=45)
        self.slider_R.pack(side=tk.RIGHT, fill=tk.X, expand=tk.TRUE, padx=30)

        savebutton_frame = tk.Frame(self)
        savebutton_frame.pack(side=tk.BOTTOM, fill=tk.NONE)

        filename_var = tk.StringVar(self, value='leap_hands_transform.yaml')

        self.filename_label = tk.Label(savebutton_frame, text="filename", fg="black")
        self.filename_label.pack(side=tk.LEFT, fill=tk.X, padx=10)

        self.filename = tk.Entry(savebutton_frame, textvariable=filename_var, width=40)
        self.filename.pack(side=tk.LEFT, fill=tk.NONE, expand=tk.FALSE, padx=30)

        self.save_button = tk.Button(savebutton_frame, text="Save", command=self.onSaveButtonPressed)
        self.save_button.pack(side=tk.LEFT, pady=30)
        # tk.IntVar()
        # self.quit.pack(side="bottom")

    def value_changed(self):
        return [self.slider_Y.get(), self.slider_P.get(), self.slider_R.get()] != self.rpy

    def get_YPR(self):
        self.rpy = [self.slider_Y.get(), self.slider_P.get(), self.slider_R.get()]
        return self.rpy

    def get_source(self):
        return self.source.get()

    def get_target(self):
        return self.target.get()

    def get_filename(self):
        return self.filename.get()

    def onSaveButtonPressed(self):
        y,p,r = self.get_YPR()
        rpy = [r*math.pi/180, p*math.pi/180, y*math.pi/180]
        translation = [0.0, 0.0, 0.0]
        quaternion = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2],'sxyz').tolist()
        data = dict()
        data['translation'] = translation
        data['orientation'] = quaternion
        data['target'] = self.get_target()
        data['source'] = self.get_source()
        try:
            RSP = rospkg.RosPack()
            path = RSP.get_path('relaxed_leap_teleop') + '/config/'
        except rospkg.ResourceNotFound:
            path = ''
        print('saving file as: ', path + self.filename.get(), '...')
        with open(path + self.get_filename(), 'w') as ff:
            documents = yaml.dump(data, ff, default_flow_style=False)
        print('saving complete!')


def controlled_transform():
    global rpy
    global source_frame
    global target_frame
    if app.value_changed():
        y,p,r = app.get_YPR()
        rpy = [r*math.pi/180, p*math.pi/180, y*math.pi/180]
        source_frame = app.get_source()
        target_frame = app.get_target()
        print(map(lambda x: '%.3f' % x, tf.transformations.quaternion_from_euler(*rpy)), source_frame, target_frame)
        # print(map(lambda x: '%.3f' % x, rpy), source_frame, target_frame)
    static_transform(rpy, source=source_frame,target=target_frame)
    app.after(1,controlled_transform)

if __name__ == '__main__':

    rpy = [0.,0.,0.]
    rospy.init_node('transformer')
    root = tk.Tk()
    root.title("Angles of the transform")
    root.geometry("720x400")
    app = GUI(master=root)
    source_frame = app.get_source()
    target_frame = app.get_target()
    app.after(0,controlled_transform)
    # app.after_idle(controlled_transform)
    
    app._poll_job_id = controlled_transform
    app.mainloop()

    # while not rospy.is_shutdown():
    #     controlled_transform()
