#! /usr/bin/env python


from __future__ import print_function, division, absolute_import
from math import degrees
import numpy as np
from copy import deepcopy

# from trac_ik_python.trac_ik import IK 
from visualization_msgs.msg import Marker
import rospy
from rospkg import RosPack
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from RelaxedIK.relaxedIK import 
from relaxed_ik.msg import EEPoseGoals, JointAngles
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_commander.conversions import list_to_pose
from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_multiply, quaternion_inverse, quaternion_from_euler
import tf2_ros
import tf 

from functools import partial

from teleoperation_ur5_allegro_leap.teleop.app import UR5_EE_GUI

jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

posegoal = EEPoseGoals()
posegoal.header.frame_id = 'world'
jangles = JointState()
jangles.name = jnames
position = [0.] * 3
orientation = ([0.] * 3) + [1]

def onSolutionReceived(joint_angle_msg):
    # print(joint_angle_msg.angles.data)
    jangles.position = joint_angle_msg.angles.data
    joint_target_publisher.publish(jangles)

def ypr2quat(y, p, r, degrees=False):
    rot = Rotation.from_euler('xyz', (y, p, r), degrees=degrees)
    return rot.as_quat().tolist()

def send_control( T=0.3, p0=np.r_[0., 0., 0., 0., 0., 0., 1.]):
    global posegoal
    global orientation
    global position
    global gui
    global ti
    global world_p0_list
    pose = (np.asarray(position) - p0[:3]).tolist() + quaternion_multiply(orientation, p0[3:]).tolist()
    pose_from_world = np.asarray((np.asarray(pose[:3]) + np.asarray(world_p0_list[:3])).tolist() \
        + quaternion_multiply(world_p0_list[3:], pose[3:]).tolist())
    
    
    posegoal.ee_poses[0] = list_to_pose(pose)
    goal_pub.publish(posegoal)
    ti += 1
    # position[2] = f(tk/0.3)
    if gui.value_changed():
        # orientation = ypr2quat(*gui.get_YPR(), degrees=True)
        orientation = gui.get_quat()
        position = gui.get_XYZ()
        # print(pose[:3], pose[3:])
        print(pose_from_world[:3].round(3).tolist(), pose_from_world[3:].round(3).tolist())
    if not rospy.is_shutdown():
        gui.after(int(T * 1000), lambda: send_control(T, p0))
    
if __name__ == '__main__':
    rospy.init_node('move_ur5_relaxed')
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1), True)
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    joint_target_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)
    goal_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
    rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, onSolutionReceived, queue_size=10)
    p = list_to_pose(6*[0.] + [1.]) # initial position
    posegoal.ee_poses.append(p)
    goal_pub.publish(posegoal) # go to initial position
    rospy.sleep(0.1)
    p0_transform = tf_buffer.lookup_transform('hand_root', 'world', rospy.Time(0))
    world_p0_list = [p0_transform.transform.translation.x, p0_transform.transform.translation.y, p0_transform.transform.translation.z] +\
        [p0_transform.transform.rotation.x, p0_transform.transform.rotation.y, p0_transform.transform.rotation.z, p0_transform.transform.rotation.w]
    world_p0_list = np.asarray(world_p0_list)
    quat_correction = quaternion_multiply(quaternion_inverse(world_p0_list[3:]), [0.707, 0.000, -0.707, 0.000])
    print(world_p0_list[:3].round(3), world_p0_list[3:].round(3))
    # print(quaternion_multiply(world_p0_list[3:], quat_correction))
    
    gui = UR5_EE_GUI.as_standalone()
    T = 0.1 
    ti = 0
    # p0 = np.r_[-0.5, 0., 0., 0., 0., 0., 1.][ 0.66477464, -0.17582887, -0.7260532 , -0.00237526]
    # p0 = np.r_[0.0, 0., 0., -0.66477464,  0.17582887, -0.7260532 , -0.00237526]
    p0 = np.asarray([-.20, 0., 0.] + quat_correction.tolist())
    # p0 = np.asarray([0.0]*6 + [1.])
    
    gui.after(int(T * 1000), lambda: send_control(T, p0=p0))
    gui.mainloop()
    
    