#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
from rospkg import RosPack, ResourceNotFound
import yaml
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from leap_motion.msg import leapros, Human, Hand, Finger, Bone


def rotmat_from_hand(direction, normal):
    normal = np.array(normal)
    direction = np.array(direction)
    # palm_center = np.array(palm_center)


    handXBasis = np.cross(normal, direction/np.linalg.norm(direction, 2))
    handYBasis = -normal
    handZBasis = -direction
    # handOrigin = palm_center
    new_basis = np.array([handXBasis, handYBasis, handZBasis])
    old_basis = np.diag([1., 1., 1.])
    R = np.linalg.inv(new_basis).dot(old_basis)
    R = np.concatenate([R,np.array([[0,0,0]])],axis=0)
    R = np.concatenate([R,np.array([[0,0,0,1]]).T],axis=1)
    
    return R
    

def get_hand_quaternion(direction, normal):
    R = rotmat_from_hand(direction, normal)
    return tf.transformations.quaternion_from_matrix(R)

def advertise_tf(xyz, quat, parent, child, tf_time, tf_pub):
    Stransform = TransformStamped()
    Stransform.header.stamp = tf_time
    Stransform.header.frame_id = parent
    Stransform.child_frame_id = child
    Stransform.transform.translation.x = xyz[0]
    Stransform.transform.translation.y = xyz[1]
    Stransform.transform.translation.z = xyz[2]
    Stransform.transform.rotation.x = quat[0]
    Stransform.transform.rotation.y = quat[1]
    Stransform.transform.rotation.z = quat[2]
    Stransform.transform.rotation.w = quat[3]
    tf_pub.sendTransform(Stransform)

def read_from_yaml(filename='leap_hands_transform.yaml'):
    RSP = RosPack()
    try:
            RSP = RosPack()
            path = RSP.get_path('relaxed_leap_teleop') + '/config/'
    except ResourceNotFound:
        path = ''
    with open(path + filename, 'r') as ff:
        resource_dict = yaml.load(ff)
        parent = resource_dict['source']
        child = resource_dict['target']
        quat = resource_dict['orientation']
        xyz = resource_dict['translation']
        print('saving complete!')
    print('reading file as: ', path + filename, '...')

    return xyz, quat, parent, child
class Leap_TF_Pub(object):

    def __init__(self, leap_topic='/leap_motion/leap_device'):

        self.leap_topic = leap_topic
        self.leap_listener = None
        self.tf_pub = tf2_ros.TransformBroadcaster()
        rospy.on_shutdown(self.shutdown)
        self.start_listen()

    def __OnLeapMessageReceived(self, leap_human):

        hands = [leap_human.left_hand, leap_human.right_hand]
        self.process_hand(leap_human.left_hand, 'left_')
        self.process_hand(leap_human.right_hand, 'right_')

    def process_hand(self, leap_hand, hand_str):
        # leap_hand = Hand()
        if leap_hand.is_present:
            palm_position = leap_hand.palm_center
            hand_xyz = [palm_position.x, palm_position.y, palm_position.z]
            direction = [leap_hand.direction.x, leap_hand.direction.y, leap_hand.direction.z, ]
            normal = [leap_hand.normal.x, leap_hand.normal.y, leap_hand.normal.z, ]
            #szxy
            hand_quat = get_hand_quaternion(direction, normal)
            advertise_tf(hand_xyz, hand_quat, 'leap_hands', hand_str + 'leap_hand_center', leap_hand.header.stamp, self.tf_pub)
            advertise_tf(leap_hand.wrist_position, hand_quat, 'leap_hands', hand_str + 'leap_hand_wrist', leap_hand.header.stamp, self.tf_pub)
            for finger in leap_hand.finger_list:
                finger_name = self.finger_names[finger.type]
                finger.header.stamp = leap_hand.header.stamp
                # print(finger_name)
                self.process_finger(finger, hand_str + finger_name + "_")


    def process_finger(self, leap_finger, finger_str):
        # leap_finger = Finger()
        finger_name = self.finger_names[leap_finger.type]
        for bone_idx, bone in enumerate(leap_finger.bone_list):
            bone.header.stamp = leap_finger.header.stamp
            self.process_bone(bone, finger_str)

    def process_bone(self, leap_bone, bone_str):
        # leap_bone = Bone()
        boneend_xyz = [leap_bone.bone_end.position.x, leap_bone.bone_end.position.y, leap_bone.bone_end.position.z]
        bonestart_xyz = [leap_bone.bone_start.position.x, leap_bone.bone_start.position.y, leap_bone.bone_start.position.z]
        quat = [0.0, 0.0, 0.0, 1.0]
        rounder = lambda x: round(x,3)
        # print(leap_bone.type, self.bone_names[leap_bone.type])
        if self.bone_names[leap_bone.type] in self.bone_names[1:] :
                advertise_tf(boneend_xyz, quat, 'leap_hands', bone_str + '' + str(leap_bone.type), leap_bone.header.stamp, self.tf_pub)
        else:
            advertise_tf(bonestart_xyz, quat, 'leap_hands', bone_str + 'base', leap_bone.header.stamp, self.tf_pub)
            #advertise_tf(boneend_xyz, quat, 'leap_hands',  bone_str + '0', leap_bone.header.stamp, self.tf_pub)

    @property
    def finger_names(self):
        return ["Thumb", "Index", "Middle", "Ring", "Pinky"]

    @property
    def bone_names(self):
        return [
            #'Metacarpal',
            'Proximal', 'Intermediate', 'Distal', 'Tip']

    def shutdown(self):
        rospy.loginfo('stop listening....')
        self.leap_listener.unregister()

    def start_listen(self):
        rospy.loginfo('>>> Subscriber Started!')
        self.leap_listener = rospy.Subscriber(self.leap_topic, Human, self.__OnLeapMessageReceived, queue_size=1)



if __name__=="__main__":

    rospy.init_node("leap2tf")
    leap_tf = Leap_TF_Pub('/leap_motion/leap_filtered')
    configfile = rospy.get_param('~transform_file', default='leap_hands_transform.yaml')
    xyz, quat, parent, child = read_from_yaml(configfile)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        advertise_tf(xyz, quat, parent, child, rospy.Time.now(), leap_tf.tf_pub)
        rate.sleep()

    # rospy.spin()