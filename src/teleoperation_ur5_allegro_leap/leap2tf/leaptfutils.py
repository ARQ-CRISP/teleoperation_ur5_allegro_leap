import tf
import numpy as np
import yaml

from rospkg import RosPack, ResourceNotFound
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseArray, Pose
from tf.transformations import quaternion_multiply, quaternion_conjugate


def rotate_vec_quat(q, v):
    return quaternion_multiply(
        quaternion_multiply(q, np.r_[v, 0.]), 
        quaternion_conjugate(q))[:3]

def rotate_quat(q1, q2):
    return np.r_[rotate_vec_quat(q1, q2[:3]), q2[3]]


def quaternion_from_2vectors(u, v):

    w = np.cross(u, v)
    q = [w[0], w[1], w[2], np.dot(u, v)]
    mag = np.linalg.norm(q, 2)
    q[3] += mag
    q /= np.linalg.norm(q, 2)
    return q.tolist()

def quaternion_from_axis_angle(v, theta):

    return np.r_[v.squeeze() * np.sin(theta/2), np.cos(theta/2)]


def rotmat_from_hand(direction, normal):
    normal = np.array(normal)
    direction = np.array(direction)
    # palm_center = np.array(palm_center)

    direction /= np.linalg.norm(direction, 2)
    # handXBasis = np.cross(normal, direction)
    # handYBasis = -normal
    # handZBasis = -direction
    handBasis = [
        direction,
        -normal,
        np.cross(normal, direction),
        ]

    # handOrigin = palm_center
    R = basis_change(handBasis)
    
    return R

def basis_change(new_basis, old_basis=np.diag([1., 1., 1.])):
    new_basis = np.array(new_basis)
    # old_basis = np.diag([1., 1., 1.])
    R = np.linalg.inv(new_basis).dot(old_basis)
    R = np.concatenate([R,np.array([[0,0,0]])],axis=0)
    R = np.concatenate([R,np.array([[0,0,0,1]]).T],axis=1)
    return R
    

def get_hand_quaternion(direction, normal):
    R = rotmat_from_hand(direction, normal)
    quat  = tf.transformations.quaternion_from_matrix(R)
    return quat / np.linalg.norm(quat,2)


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

def advertise_pose(pose, parent, time, pose_pub):
    Spose = PoseStamped()
    Spose.header.stamp = time
    Spose.header.frame_id = parent
    Spose.pose.position.x = pose[0][0]
    Spose.pose.position.y = pose[0][1]
    Spose.pose.position.z = pose[0][2]
    Spose.pose.orientation.x = pose[1][0]
    Spose.pose.orientation.y = pose[1][1]
    Spose.pose.orientation.z = pose[1][2]
    Spose.pose.orientation.w = pose[1][3]
    pose_pub.publish(Spose)

def advertise_parray(poses, parent, time, pose_pub):
    Parray = PoseArray()
    Parray.header.stamp = time
    Parray.header.frame_id = parent
    for pose in poses:
        new_pose = Pose()
        new_pose.position.x = pose[0][0]
        new_pose.position.y = pose[0][1]
        new_pose.position.z = pose[0][2]
        new_pose.orientation.x = pose[1][0]
        new_pose.orientation.y = pose[1][1]
        new_pose.orientation.z = pose[1][2]
        new_pose.orientation.w = pose[1][3]
        Parray.poses.append(new_pose)
    pose_pub.publish(Parray)

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