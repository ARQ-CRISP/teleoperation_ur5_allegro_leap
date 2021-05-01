#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppolaqmul.ac.uk
last update: 26/02/21
'''
from __future__ import print_function, division, absolute_import
import numpy as np
from copy import deepcopy

# from trac_ik_python.trac_ik import IK 
from visualization_msgs.msg import Marker
import rospy
from rospkg import RosPack
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_commander.conversions import list_to_pose
from scipy.spatial.transform import Rotation
current_ur_state = None
subscriber = None
joint_target_publisher = None
# ik_solver = None
def OnJointStateReceived(msg):
    global current_ur_state
    global joint_target_publisher
    # msg = JointState()
    joint_idx = [i for i, name in enumerate(msg.name) if 'wrist' in name or 'elbow' in name or 'shoulder' in name]
    # print([(msg.name[i], msg.position[i]) for i in joint_idx])
    current_ur_state = JointState()
    current_ur_state.header = msg.header
    current_ur_state.name = [msg.name[i] for i in joint_idx]
    current_ur_state.position = [msg.position[i] for i in joint_idx]
    # print([msg.name[i] for i in joint_idx])
    

    
if __name__ == '__main__':
    init_joints = [-0.9928200048353305, -1.2833503113491043, 1.7950417794705156, 4.200635272987568, -1.5708741632014989, -0.20274482729631219]
    
    rospy.init_node('ur_controller')
    arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    relaxed_ik_path = RosPack().get_path('relaxed_ik')
    pub_marker = rospy.Publisher('ee_pose', Marker)
    RIK_solver = get_relaxedIK_from_info_file(relaxed_ik_path +'/src', 'ur5_allegro_info.yaml', preconfig=True)
    RIK_solver.vars.init_state = init_joints
    RIK_solver.vars.relaxedIK_vars_update(init_joints)
    # ik_solver = IK("world",
    #            "hand_root")
    joint_target_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)
    # subscriber = rospy.Subscriber('/joint_states', JointState, OnJointStateReceived, queue_size=1)
    # ik_solver.
    position = [0., 0., 0.]
    quat = [0., 0., 0., 1.]
    rate = rospy.Rate(30)
    joints = init_joints
    base_frame = (0.145, 0., 1.656)
    base_quat = (0.5, 0.5, 0.5, 0.5)
    base_m = Rotation.from_quat(base_quat).as_dcm()
    while not rospy.is_shutdown():
        position[0] = 2/10 * np.cos(2*np.pi*1.*rospy.Time.now().to_sec())
        joints = RIK_solver.solve([position], [quat])
        # init_joints = None
        print(RIK_solver.vars.ee_positions[0])
        out_m = Marker()
        target = JointState()
        out_m.type = Marker.SPHERE
        out_m.scale.x = out_m.scale.y = out_m.scale.z = 0.1 
        out_m.scale.z *= 2
        out_m.scale.y *= 1/2
        out_m.color.a = 0.8
        out_m.color.r = 0.7
        out_m.header.frame_id = 'hand_root'
        position = RIK_solver.vars.ee_positions[0]
        # position = np.matmul((base_m), position) - base_frame
        out_m.pose = list_to_pose(position.tolist()+quat)
        # out_m.pose.position.x += base_frame[0]
        # out_m.pose.position.y += base_frame[1]
        # out_m.pose.position.z += base_frame[2]
        target.position = joints#init_joints
        target.name = arm_joints
        pub_marker.publish(out_m)
        RIK_solver.vars.collision_graph.c.draw_all()
        joint_target_publisher.publish(target)
        # if current_ur_state is not None:
            # target = JointState()
            # if current_ur_state is not None:
            #     target = deepcopy(current_ur_state)
            #     target.position[-1] = np.pi/2 * np.cos(2*np.pi*.1*rospy.Time.now().to_sec())
            #     joint_target_publisher.publish(target)
        rate.sleep()
        