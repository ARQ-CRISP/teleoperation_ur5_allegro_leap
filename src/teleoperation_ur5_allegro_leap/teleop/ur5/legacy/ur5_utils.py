import rospy
import numpy as np
import tf.transformations as T



def get_qrotation(theta, axis):
        ax = np.diag(np.ones(3))[axis]
        c = np.cos(theta/2)
        s = np.sin(theta/2)
        q = np.array((ax * s).tolist() + [c])
        q /= np.linalg.norm(q)  
        return q

def combine_rotations(q1, q2):
    return T.quaternion_multiply(q2, q1)


def rotate_quaternion(q, rotations):
    R = [0.0, 0.0, 0.0, 1.0] 
    for r in rotations:
        combine_rotations(R, r)

    return combine_rotations(q, R)    