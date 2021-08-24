#!/usr/bin/env python
from __future__ import print_function, division
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
import tf
import tf2_ros
# import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from leap_motion.msg import  Human #, leapros, Hand, Finger, Bone
from tf.transformations import quaternion_from_matrix, quaternion_multiply #, quaternion_from_matrix, quaternion_matrix
from leaptfutils import advertise_tf, basis_change, quaternion_from_axis_angle #, get_hand_quaternion,  rotate_quat, rotate_vec_quat


class Leap_TF_Pub(object):

    __center_name = 'leap_hand_center'
    __wrist_name = 'leap_hand_wrist'
    __side_strings = ['left_', 'right_']
    __base_orient = [ 0.5, -0.5,  0.5, -0.5]

    def __init__(self,
                 leap_topic='/leap_motion/leap_device', hand_root='wrist',
                 hands_frame_name='leap_hands', fingers_frame_base_name='leap_fingers',
                 orient_all_bones=False):

        __root_joint_possibilities = {
            'center': self.__center_name, 'wrist': self.__wrist_name}
        self.leap_topic = leap_topic
        self.leap_listener = None
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.hand_root = __root_joint_possibilities[hand_root]
        # self.publish_on = publish_on  # can be 'root' or 'fixed_frame'
        self.hands_frame = hands_frame_name
        self.fingers_frame_base_name = fingers_frame_base_name
        self.bones_to_orient = 0 if orient_all_bones is True else 2
        rospy.on_shutdown(self.shutdown)
        self.start_listen()

    @property
    def root_coordinate(self):
        return [self.root_xyz.tolist(), self.root_quat.tolist()]

    @root_coordinate.setter
    def root_coordinate(self, new_coord):
        self.root_xyz, self.root_quat = np.array(
            new_coord[0]), np.array(new_coord[1])

    def __OnLeapMessageReceived(self, leap_human):

        hands = [leap_human.left_hand, leap_human.right_hand]
        for side_string, hand in zip(self.__side_strings, hands):
            self.process_hand(hand, side_string)

    def process_hand(self, leap_hand, side_str):

        if leap_hand.is_present:
            palm_position = leap_hand.palm_center
            center_xyz = [palm_position.x, palm_position.y, palm_position.z]
            wrist_xyz = [
                leap_hand.arm.wrist.position.x,
                leap_hand.arm.wrist.position.y,
                leap_hand.arm.wrist.position.z]

            direction = np.array([leap_hand.direction.x,
                         leap_hand.direction.y, leap_hand.direction.z, ])
            normal = np.array([leap_hand.normal.x,
                      leap_hand.normal.y, leap_hand.normal.z, ])

            binormal = np.cross(direction,normal)

            # basis = [direction, -normal, -binormal]
            basis = [normal, binormal, direction]
            
            
            R = basis_change(basis)
            q = tf.transformations.quaternion_from_matrix(R)
            q /= np.linalg.norm(q)
            # angleq1 = np.array([ -0.7068252, 0, 0, 0.7073883 ])
            # angleq2 = np.array([ 0, 0.7068252, 0, 0.7073883 ])
            # angleq = tf.transformations.quaternion_multiply(angleq2, angleq1)
            # angleq = np.array([ -0.5, 0.5, 0.5, 0.5 ])
            # [ 0.5, -0.5,  0.5, -0.5]
            # angleq = np.array([-0.5,0.5,-0.5,0.5])
            # angleq = np.array([0.5, 0.5, -0.5, 0.5])
            # q=tf.transformations.quaternion_multiply(angleq, q)
            # self.direction = direction
            hand_quat = q #get_hand_quaternion(direction, normal).tolist()
            base_quat = [0.0, 0.0, 0.0, 1.0]
            # base_quat = self.__base_orient
            # hand_rpy = [leap_hand.roll, leap_hand.pitch, -leap_hand.yaw] #szxy
            # print([round(q,3) for q in hand_quat])
            if 'wrist' in self.hand_root:
                self.root_coordinate = [wrist_xyz, hand_quat]
                wrist_xyz = self.root_coordinate[0]
                center_xyz = self.to_hand_reference(center_xyz)[0]
                wrist_quat = self.root_coordinate[1]
                center_quat = base_quat
                center_source_frame = side_str + self.fingers_frame_base_name
                wrist_source_frame = self.hands_frame

            elif 'center' in self.hand_root:
                self.root_coordinate = [center_xyz, hand_quat]
                center_xyz = self.root_coordinate[0]
                wrist_xyz = self.to_hand_reference(wrist_xyz)[0]
                center_quat = self.root_coordinate[1]
                wrist_quat = base_quat
                center_source_frame = self.hands_frame
                wrist_source_frame = side_str + self.fingers_frame_base_name
            
            else:
                raise ValueError("Error: the hand root has been mistakenly set up! you can chose 'wrist' or 'center'")
                # rospy.logerr()

            advertise_tf(wrist_xyz, wrist_quat, wrist_source_frame, side_str +
                         self.__wrist_name, leap_hand.header.stamp, self.tf_pub)
            advertise_tf(center_xyz, center_quat, center_source_frame, side_str +
                         self.__center_name, leap_hand.header.stamp, self.tf_pub)

            for finger in leap_hand.finger_list:
                finger_name = self.finger_names[finger.type]
                finger.header.stamp = leap_hand.header.stamp
                # print(finger_name)
                self.process_finger(finger, side_str, finger_name + "_")
            # rospy.loginfo(rospy.get_name() + '-'*50)

    def process_finger(self, leap_finger, side_str, finger_str):
        # leap_finger = Finger()
        finger_name = self.finger_names[leap_finger.type]
        for bone_idx, bone in enumerate(leap_finger.bone_list):
            bone.header.stamp = leap_finger.header.stamp
            # if not leap_finger.type == 0:
            self.process_bone(bone, side_str, finger_str, leap_finger.bone_list[0].bone_end.position)
            # self.legacy_process_bone(bone, side_str, finger_str,
            #                   leap_finger.type == 0)


    def process_bone(self, leap_bone, side_str, finger_str, finger_origin):
        source_frame_name = side_str + self.fingers_frame_base_name
        bone_str = side_str + finger_str
        boneend_xyz, bone_quat = self.to_hand_reference(
            [leap_bone.bone_end.position.x, leap_bone.bone_end.position.y, leap_bone.bone_end.position.z],
            [leap_bone.bone_end.orientation.x, leap_bone.bone_end.orientation.y, leap_bone.bone_end.orientation.z, leap_bone.bone_end.orientation.w])
        bonestart_xyz = self.to_hand_reference(
            [leap_bone.bone_start.position.x, leap_bone.bone_start.position.y, leap_bone.bone_start.position.z])[0]
        finger_origin_xyz = self.to_hand_reference(
            [finger_origin.x, finger_origin.y, finger_origin.z])[0]
        
        origin_direction = finger_origin_xyz / np.linalg.norm(finger_origin_xyz, 2)
        bone_direction = boneend_xyz - bonestart_xyz
        bone_direction /= np.linalg.norm(bone_direction, 2)
        bone_quat = [0., 0., 0., 1.]

        if leap_bone.type == 0:
            advertise_tf(bonestart_xyz, [0., 0., 0., 1.], source_frame_name,
                         bone_str + 'base', leap_bone.header.stamp, self.tf_pub)
        else:
            # bone_quat = self.quat_from_directions(origin_direction, bone_direction)
            bone_quat = self.quat_from_directions(np.eye(3)[0], bone_direction)
            if 'Thumb' not in finger_str:
                x_quat = quaternion_from_axis_angle(np.eye(3)[0], np.pi)
                bone_quat = quaternion_multiply(bone_quat, x_quat)
                # bone_quat = quaternion_from_axis_angle(bone_direction, -np.pi)
                # bone_quat = quaternion_multiply(bone_quat, [ 0, 0.3826834, 0, 0.9238795 ]) #45deg
                # bone_quat = quaternion_multiply(bone_quat, [ 0, 0.258819, 0, 0.9659258 ])  #30deg
                # bone_quat = quaternion_multiply(bone_quat, [ 0, 0.1305262, 0, 0.9914449 ]) #15deg
                # bone_quat = quaternion_multiply(bone_quat, [ 0, 0.0871557, 0, 0.9961947 ]) #10deg
            else:
                M = np.eye(4)
                n = np.cross(-bone_direction, -boneend_xyz/np.linalg.norm(boneend_xyz))
                M[0, :-1] = - bone_direction
                M[1, :-1] = -boneend_xyz/np.linalg.norm(boneend_xyz)
                M[2, :-1] = n / np.linalg.norm(n)
                if False and np.linalg.norm(n) > 1e-6 : 
                    bone_quat = quaternion_from_matrix(M)
                    bone_quat /= np.linalg.norm(bone_quat)
                else:
                    x_quat = quaternion_from_axis_angle(np.eye(3)[0], np.pi/10.)
                    bone_quat = quaternion_multiply(bone_quat, x_quat)
                # bone_quat = quaternion_multiply(bone_quat, [ 0, -0.3826834, 0, 0.9238795 ]) #-45deg
                # bone_quat = quaternion_multiply(bone_quat, [ 0, 0.3826834, 0, 0.9238795 ]) #45deg
                # bone_quat = quaternion_multiply(bone_quat, [ 0, 0.258819, 0, 0.9659258 ])  #30deg
                # bone_quat = quaternion_multiply(bone_quat, [ 0, 0.1305262, 0, 0.9914449 ]) #15deg
                # bone_quat = quaternion_multiply(bone_quat, [ 0, 0.0871557, 0, 0.9961947 ]) #10deg


        # print("{:.3f}, {:.3f}, {:.3f} -> {}".format(*(r.as_euler('xyz')/np.pi*180).tolist() + [bone_str]))
        advertise_tf(boneend_xyz, bone_quat, source_frame_name, bone_str +
                     '' + str(leap_bone.type), leap_bone.header.stamp, self.tf_pub)
        


    # def legacy_process_bone(self, leap_bone, side_str, finger_str, isthumb=False):
    #     # leap_bone = Bone()
    #     source_frame_name = side_str + self.fingers_frame_base_name

    #     bone_str = side_str + finger_str

    #     boneend_xyz = self.to_hand_reference(
    #         [leap_bone.bone_end.position.x, leap_bone.bone_end.position.y, leap_bone.bone_end.position.z])
    #     bonestart_xyz = self.to_hand_reference(
    #         [leap_bone.bone_start.position.x, leap_bone.bone_start.position.y, leap_bone.bone_start.position.z])
    #     bone_quat = [0.0, 0.0, 0.0, 1.0]
    #     bone_direction = boneend_xyz - bonestart_xyz
    #     bone_direction /= np.linalg.norm(bone_direction, 2)
    #     # rounder = lambda x: round(x,3)
    #     if leap_bone.type > 0:
    #         # bone_normal = self.get_bone_normal(isthumb, bone_direction)
    #         # bone_binormal = np.cross(bone_direction, bone_normal)
    #         # basis = [-bone_normal, -bone_binormal, bone_direction]
    #         # basis = [bone_direction, bone_binormal, -bone_normal]
    #         # R = basis_change(basis)
    #         # q = tf.transformations.quaternion_from_matrix(R)
    #         # q /= np.linalg.norm(q)
    #         # bone_direction = rotate_vec_quat(v=bone_direction, q=[0., 0., 1., 0.])
    #         bone_quat = self.quat_from_directions(np.eye(3)[0], bone_direction)
    #         # bone_quat = rotate_quat([0., 0., 1.,  0.], bone_quat)
    #         r = R.from_quat(bone_quat)
    #         print("{:.3f}, {:.3f}, {:.3f} -> {}".format(*(r.as_euler('xyz')/np.pi*180).tolist() + [bone_str]))
    #         # bone_quat = get_hand_quaternion(bone_direction, bone_normal)
    #     if leap_bone.type == 0:
    #         advertise_tf(bonestart_xyz, [0., 0., 0., 1.], source_frame_name,
    #                      bone_str + 'base', leap_bone.header.stamp, self.tf_pub)
    #     advertise_tf(boneend_xyz, bone_quat, source_frame_name, bone_str +
    #                  '' + str(leap_bone.type), leap_bone.header.stamp, self.tf_pub)

    def quat_from_directions(self, u, v):
        q = np.ones(4)
        n = np.cross(u, v)                           
        theta = np.arctan2(np.linalg.norm(n), u.dot(v))
        n /= np.linalg.norm(n)
        q = np.asarray(3*[np.sin(theta/2)] + [np.cos(theta/2)])
        q[:3] *= n
        return q

    def get_bone_normal(self, isthumb, bone_direction):
        if not isthumb:  # all but the Thumb
            r = R.from_euler('y', -90, degrees=True)
            bone_normal = r.apply(bone_direction)
        else:  # thumb
            bone_normal = -np.array([1., 0., 0])
            bone_normal -= bone_normal.dot(bone_direction) * bone_direction
            bone_normal /= np.linalg.norm(bone_normal, 2)
        return bone_normal

    def to_hand_reference(self, point_xyz, pose_quat=(0., 0., 0., 1.)):  # , ref_v, ref_q):

        # ref_v = np.array(ref_v)
        # ref_q = np.array(ref_q)
        point_xyz = np.array(point_xyz)
        d = point_xyz - self.root_xyz
        # d = [v_i - v_r for v_r, v_i in zip(ref_v, point_xyz)]
        point_xyz = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_conjugate(self.root_quat),
            tf.transformations.quaternion_multiply(np.concatenate([d, [0.0]]), self.root_quat))
        pose_quat = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_conjugate(self.root_quat),
            tf.transformations.quaternion_multiply(pose_quat, self.root_quat))
        
        return point_xyz[0:3], pose_quat

    @property
    def finger_names(self):
        return ["Thumb", "Index", "Middle", "Ring", "Pinky"]

    @property
    def bone_names(self):
        return [
            # 'Metacarpal',
            'Proximal', 'Intermediate', 'Distal', 'Tip']

    def shutdown(self):
        rospy.loginfo('stop listening....')
        self.leap_listener.unregister()

    def start_listen(self):
        rospy.loginfo('>>> Subscriber Started!')
        self.leap_listener = rospy.Subscriber(
            self.leap_topic, Human, self.__OnLeapMessageReceived, queue_size=1)


if __name__ == "__main__":

    rospy.init_node("leap2tf")

    # configfile = rospy.get_param(
    # '~transform_file', default='leap_hands_transform.yaml')
    # xyz, quat, parent, child = read_from_yaml(configfile)

    hand_root = rospy.get_param('~hand_root', default='center')
    publish_on = rospy.get_param('~publish_on', default='root')
    filtered = rospy.get_param('~filter', default=True)
    topic = '/leap_motion/leap_filtered' if filtered else '/leap_motion/leap_device'
    fixed_frame_name = rospy.get_param(
        '~fixed_frame_name', default='leap_hands')
    leap_tf = Leap_TF_Pub(leap_topic=topic,
                          hand_root=hand_root,
                          hands_frame_name='leap_hands', fingers_frame_base_name='leap_fingers',)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # advertise_tf(xyz, quat, parent, child,
                    #  rospy.Time.now(), leap_tf.tf_pub)
        rate.sleep()

    # rospy.spin()
