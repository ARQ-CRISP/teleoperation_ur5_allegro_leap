#!/usr/bin/env python
from __future__ import print_function, division, absolute_import

import numpy as np
import kdl_parser_py.urdf as kdl_parser
import PyKDL as kdl
from tf_conversions import toMsg
from collections import OrderedDict
from scipy.optimize import minimize

import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


class UR5KDL(object):
    world_frame = 'world'
    wrist_frame = 'hand_root'
    palm_frame = 'palm_link'
    
    def __init__(self, robot_description_param="robot_description"):
        
        (status, self.tree) = kdl_parser.treeFromParam(robot_description_param)
        self.bounds = [(-np.pi, np.pi) for i in range(5)]
        # self.bounds[3*4 + 1] = (-np.pi/8, np.pi/8)
        # for i in range(3):
        #     self.bounds[i*4] = (-np.pi/8, np.pi/8)
        self.chain = self.tree.getChain("world", self.wrist_frame)
        
        self._fk_solver = kdl.ChainFkSolverPos_recursive(self.chain) 
        
        self.last_sol = None
        
    def solve(self, theta):
        ee_pose = kdl.Frame()
        if type(theta) in [list, np.ndarray, tuple]:
            angles = kdl.JntArray(len(theta))
            for i, t in enumerate(theta):
                angles[i] = t
            theta = angles
        res = self._fk_solver.JntToCart(theta, ee_pose)
        return res, ee_pose
    
    def generate_pose_marker(self, ee_pose, frame_id='world'):
        
        if type(ee_pose) is list:
            ee_pose = kdl.Frame(kdl.Rotation.Quaternion(*ee_pose[1]), kdl.Vector(*ee_pose[0]))
            
        marker = Marker(pose=toMsg(ee_pose), type=Marker.ARROW, ns='/ur_target')
        marker.header.frame_id = frame_id
        marker.scale.x, marker.scale.y, marker.scale.z = 1e-2, 1e-2, 3e-2
        marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.3
        marker.color.a = 1.0
        
        return marker
            
    
    def optimize(self, x0, hand_pos, tol=1e-5):
        hand_vec = self._hand_points_to_vec(hand_pos)
        res = minimize(
            # lambda x: objfunc(x, pdist(hand_pos.reshape(4, 3), metric='cosine')),
            lambda x: self._objfunc(x, x0, hand_vec),
            x0=x0,
            method='SLSQP',
            bounds=self.bounds,
            tol=tol
            )
        self.last_sol = res
        return res
    
    def _objfunc(self, x, x0, target):
        
        # gamma = 2.5e-3
        # eps = 2e-2
        # norm_sq = np.asarray(x).dot(x)
        # diff_norm_sq = np.asarray(x).dot(x)
        
        ee_pose = self.solve(x)
        joint_diff = (x - x0).dot(x - x0)
        cartesian_diff = (target.p - ee_pose.p).dot((target.p - ee_pose.p))
        quat_diff = np.asarray(list(target.M.GetQuaternion())).dot(
            np.asarray(list(ee_pose.M.GetQuaternion())))
        return joint_diff + cartesian_diff + quat_diff
        
    

if __name__ == '__main__':
    from pprint import pprint
    rospy.init_node('test_fk')
    AKDL = UR5KDL()
    theta =  [2.3744237422943115, -2.0752771536456507, -1.7465012709247034, 
              -0.8918698469745081, 1.5678939819335938, 0.013490866869688034]
    res, pose = AKDL.solve(theta)
    
    pprint((pose.p, pose.M.GetQuaternion()))
    
    