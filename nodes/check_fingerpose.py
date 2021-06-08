#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 15/11/19
'''
from __future__ import division, print_function
from pynput.keyboard import Key, Controller, Listener, KeyCode
from rospkg.rospack import RosPack
from pickle import load
from pprint import pprint
import numpy as np
import rospy
# from tf2_geometry_msgs
import tf2_ros
from teleoperation_ur5_allegro_leap import Leap_Teleop_Allegro
from leap_motion.msg import Human
from moveit_commander.conversions import list_to_pose, list_to_pose_stamped, pose_to_list, transform_to_list, list_to_transform

def leap_callback(msg):
    # msg = Human()
    global events
    stamp = msg.header.stamp
    leap_hand = msg.left_hand if lefthand_mode else msg.right_hand
    current_fingertips = dict()

    # print(msg.left_hand.is_present, msg.right_hand.is_present)
    if leap_hand.is_present:
        for name in ['Index', 'Middle', 'Ring']:
            tf_stamped = tf_buffer.lookup_transform(fingertip_links[name], fingertip_links['Thumb'], stamp, rospy.Duration(0.2))
            fingertip = transform_to_list(tf_stamped.transform)
            dist2thumb = np.linalg.norm(fingertip[:3])
            # print(name, fingertip_links[name], dist2thumb)
            if dist2thumb <= 1e-2:
                if len(events) == 0 or (stamp - init_time).to_sec() - events[-1][1] > .5:
                    events += [(name, round((stamp - init_time).to_sec(), 3), dist2thumb.round(3))]
                rospy.loginfo('{:s} touched Thumb at: {:.3f} -> Distance value: {:.3f}'.format(name, (stamp - init_time).to_sec(),dist2thumb))
    
def print_events():
    global events
    pprint(events)

if __name__ == "__main__":
    events = []
    rospy.on_shutdown(print_events)
    rospy.init_node('leap2allegro')
    tf_buffer = tf2_ros.Buffer(rospy.Duration(50))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    lefthand_mode = rospy.get_param('~left_hand', False)
    leap_topic = rospy.get_param('~leap_topic', '/leap_motion/leap_filtered')
    package_path = RosPack().get_path('teleoperation_ur5_allegro_leap')

    fingertips = dict([('Index', 3), ('Middle', 7), ('Ring', 11), ('Thumb', 15)])
    root_link = 'hand_root'
    fingertip_links = dict([(name, "link_{}_tip".format(fingertip)) for name, fingertip in fingertips.items()])
    init_time = rospy.Time().now()
    rospy.loginfo("start counting {:.3f}".format((rospy.Time().now() - init_time).to_sec()))
    rospy.Subscriber('/leap_motion/leap_filtered', Human, leap_callback)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     pass
        
    # current_time = rospy.Time().now()
    