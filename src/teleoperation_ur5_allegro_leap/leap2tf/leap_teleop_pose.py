#!/usr/bin/env python
from __future__ import print_function
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseArray, PoseStamped
from leap_motion.msg import leapros, Human, Hand, Finger, Bone
from leaptfutils import advertise_tf, advertise_parray, advertise_pose, get_hand_quaternion, quaternion_from_2vectors, read_from_yaml
from scipy.spatial.transform import Rotation as R

class Leap_Teleop_Pose_Publisher(object):

    __center_name = 'leap_hand_center'
    __wrist_name = 'leap_hand_wrist'
    __side_strings = ['left_', 'right_']

    def __init__(self, leap_topic='/leap_motion/leap_device', left_hand_mode=False, hand_root='wrist', parent_hand='world', parent_fingers='leap_hand_wrist'):
        
        self.left_hand_mode = left_hand_mode
        __root_joint_possibilities = {'center': self.__center_name, 'wrist': self.__wrist_name}
        self.hand_str = self.__side_strings[0] if left_hand_mode else self.__side_strings[1]
        self.leap_topic = leap_topic
        self.leap_listener = None
        self.fingers_pub = rospy.Publisher('/leap_poses/'+self.hand_str + 'fingers', PoseArray, queue_size=10)
        self.hand_pub = rospy.Publisher('/leap_poses/'+self.hand_str + 'hand', PoseStamped, queue_size=10)
        self.hand_root = __root_joint_possibilities[hand_root]
        self.parent_hand_frame = parent_hand
        self.parent_fingers_frame = parent_fingers
        # self.fixed_frame = fixed_frame_name
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo('['+rospy.get_name()[1:]+'] ' + 'hand_root: ' + hand_root)
        rospy.loginfo('['+rospy.get_name()[1:]+'] ' + 'parent_hand_frame: ' + self.parent_hand_frame)
        rospy.loginfo('['+rospy.get_name()[1:]+'] ' + 'parent_fingers_frame: ' + self.parent_fingers_frame)
        self.start_listen()
    
    def shutdown(self):
        rospy.loginfo('stop listening....')
        self.leap_listener.unregister()

    def start_listen(self):
        rospy.loginfo('>>> Subscriber Started!')
        self.leap_listener = rospy.Subscriber(self.leap_topic, Human, self.__OnLeapMessageReceived, queue_size=1)

    def change_reference(self, source_xyz, ref_pose):#, ref_v, ref_q):
    
        # ref_v = np.array(ref_v)
        # ref_q = np.array(ref_q)
        source_xyz = np.array(source_xyz)
        d = source_xyz - ref_pose[0]
        # d = [v_i - v_r for v_r, v_i in zip(ref_v, point_xyz)]
        source_xyz = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_conjugate(ref_pose[1]),
            tf.transformations.quaternion_multiply(np.concatenate([d, [0.0]]), ref_pose[1]))
        return source_xyz[0:3]

    def __OnLeapMessageReceived(self, leap_human):

        # hands = [leap_human.left_hand, leap_human.right_hand]
        leap_hand = leap_human.left_hand if self.left_hand_mode else leap_human.right_hand
        
        # for side_string, hand in zip(self.__side_strings, hands):
        # self.process_hand(hand, side_string)
        if leap_hand.is_present:
            palm_position = leap_hand.palm_center
            center_xyz = [palm_position.x, palm_position.y, palm_position.z]
            wrist_xyz = leap_hand.wrist_position
            direction = [leap_hand.direction.x, leap_hand.direction.y, leap_hand.direction.z, ]
            normal = [leap_hand.normal.x, leap_hand.normal.y, leap_hand.normal.z, ]
            hand_quat = get_hand_quaternion(direction, normal).tolist()
            base_quat = [0.0, 0.0, 0.0, 1.0]

            if 'wrist' in self.hand_root:
                self.root_coordinate = [wrist_xyz, hand_quat]
                # wrist_xyz = self.root_coordinate[0]
                center_xyz = self.change_reference(center_xyz, self.root_coordinate)
                wrist_quat = self.root_coordinate[1]
                center_quat = base_quat
                center_source_frame = self.parent_fingers_frame
                wrist_source_frame = self.parent_hand_frame

            elif 'center' in self.hand_root:

                self.root_coordinate = [center_xyz, hand_quat]
                # center_xyz = self.root_coordinate[0]
                wrist_xyz = self.change_reference(wrist_xyz, self.root_coordinate)
                center_quat = self.root_coordinate[1]
                wrist_quat = base_quat
                center_source_frame = self.parent_hand_frame
                wrist_source_frame = self.parent_fingers_frame


            advertise_pose([wrist_xyz, wrist_quat], wrist_source_frame, leap_hand.header.stamp, self.hand_pub)
            # advertise_pose([center_xyz, center_quat], center_source_frame, leap_hand.header.stamp, self.hand_pub)
            parray = []
            for finger in leap_hand.finger_list:

                tip_bone = finger.bone_list[-1]
                bonestart_xyz, boneend_xyz, bone_quat = self.process_bone(tip_bone, finger.type==0)
                parray.append([boneend_xyz, bone_quat])
            
            advertise_parray(parray, self.parent_fingers_frame, leap_hand.header.stamp, self.fingers_pub)
    
    def process_bone(self, bone, isthumb, estimate_quat=True):
        boneend_xyz = [bone.bone_end.position.x, bone.bone_end.position.y, bone.bone_end.position.z]
                
        bonestart_xyz = [bone.bone_start.position.x, bone.bone_start.position.y, bone.bone_start.position.z]

        boneend_xyz = self.change_reference(boneend_xyz, self.root_coordinate)
        bonestart_xyz = self.change_reference(bonestart_xyz, self.root_coordinate)
        if estimate_quat:
            bone_quat = self.estimate_bone_quat(boneend_xyz, bonestart_xyz, isthumb)
        else:
            bone_quat = [0., 0., 0., 1.]

        return bonestart_xyz, boneend_xyz, bone_quat

    def estimate_bone_quat(self, boneend_xyz, bonestart_xyz, isthumb):
        bone_direction = np.array(boneend_xyz) - np.array(bonestart_xyz)
        magn = np.linalg.norm(bone_direction, 2)
        bone_direction /= magn
        if not isthumb: # all but the Thumb 
            r = R.from_euler('z', -90, degrees=True)
            bone_normal = r.apply(bone_direction)
        else:
            bone_normal = -np.array([0., 1., 0])
            bone_normal -= bone_normal.dot(bone_direction) * bone_direction
            bone_normal /= np.linalg.norm(bone_normal, 2)
        bone_quat = get_hand_quaternion(bone_direction, bone_normal)
        return bone_quat

if __name__=="__main__":

    rospy.init_node("leap_teleop_pose")
    
    
    hand_root = rospy.get_param('~hand_root', default='wrist')
    publish_on = rospy.get_param('~publish_on', default='root')
    parent_fingers = rospy.get_param('~finger_parent', default='hand_root')
    parent_hand = rospy.get_param('~hand_parent', default='world')
    Leap_Teleop_Pose_Publisher(leap_topic='/leap_motion/leap_filtered',
    left_hand_mode=False,
    hand_root=hand_root,
    parent_hand=parent_hand,
    parent_fingers=parent_fingers,)


    rospy.spin()