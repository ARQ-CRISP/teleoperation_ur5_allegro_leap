#!/usr/bin/env python
from __future__ import print_function, division, absolute_import

import numpy as np
import kdl_parser_py.urdf as kdl_parser
import PyKDL as kdl
from tf_conversions import fromTf, toMsg
from collections import OrderedDict
from scipy.optimize import minimize

import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray, Marker

from .allegro_utils import allegro_finger2linklist, finger_allegro_idx, allegro_fingers

class AllegroKDL(object):
    
    finger2tip_link = OrderedDict([
        ('Index', 'link_3_tip'),
        ('Middle', 'link_7_tip'),
        ('Ring', 'link_11_tip'),
        ('Thumb', 'link_15_tip')])
    
    def __init__(self, robot_description_param="robot_description"):
        (status, self.tree) = kdl_parser.treeFromParam(robot_description_param)
        self.bounds = [(0., np.pi / 2.) for i in range(16)]
        self.bounds[3*4 + 1] = (-np.pi/8, np.pi/8)
        for i in range(3):
            self.bounds[i*4] = (-np.pi/8, np.pi/8)
        self.chains = OrderedDict([
            (finger_name, self.tree.getChain("hand_root", self.finger2tip_link[finger_name])) \
            for finger_name, value in finger_allegro_idx.items() if value is not None])
        self._fk_solvers = OrderedDict([
            (finger_name, kdl.ChainFkSolverPos_recursive(self.chains[finger_name])) \
            for finger_name in self.chains])
        self.last_sol = None
        
    def solve_finger(self, finger, theta):
        ee_pose = kdl.Frame()
        if type(theta) in [list, np.ndarray]:
            angles = kdl.JntArray(len(theta))
            for i, t in enumerate(theta):
                angles[i] = t
            theta = angles
        res = self._fk_solvers[finger].JntToCart(theta, ee_pose)
        return res, ee_pose
    
    def solve(self, theta):
        ee_poses = []
        tot_res = []
        for i, finger_name in enumerate(self.chains):
            # print(finger_name, (i*4),(i*4+4), theta[(i*4): (i*4+4)])
            # print(finger_name)
            res, sol_pose = self.solve_finger(finger_name, theta[(i*4): (i*4+4)])
            ee_poses.append(sol_pose)
            tot_res.append(res)
        # print('')
        return tot_res, ee_poses
    
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
        
    def _objfunc(self, x, x0, hand_vec):
        
        gamma = 2.5e-3
        eps = 2e-2
        norm_sq = np.asarray(x).dot(x)
        diff_norm_sq = np.asarray(x).dot(x)
        
        def s(di, S, eps=1e-3):
            if di > eps:
                return 1.0
            elif S == 0:
                return 200.0
            else:
                return 400.0
            
        def f(di, S, eps=1e-3):
            
            beta = 1.6
            eta_1 = 1e-4
            eta_2 = 3e-2
            if di > eps:
                return beta * di
            elif S == 0:
                return eta_1
            else:
                return eta_2
        
        res, poses = self.solve(x)
        allegro_pos = np.asarray([(list(pose.p)) for pose in poses])
        allegro_vec = self._hand_points_to_vec(allegro_pos)
        # print('hand vec', hand_vec)
        # print('allegro vec', allegro_vec)
        err = 0.0
        for i, (a_vec, h_vec) in enumerate(zip(allegro_vec, hand_vec)):
            d_i = np.linalg.norm(h_vec)
            diff = a_vec - f(d_i, int(i >= 3), eps) * h_vec / d_i
            err += diff.dot(diff) * s(d_i, int(i >= 3), eps)
        return gamma * norm_sq + gamma * diff_norm_sq + 1/2 * err
    
    def _hand_points_to_vec(self, pos):
        
        V = np.zeros((10, 3))
        # V = np.zeros((6, 3))
        #S1 Thumb - Finger
        # S1 = pos[-1] - pos[:-1]
        V[:3] = pos[-1] - pos[:-1]
        #S2 Finger - Finger
        V[3:5] = pos[[0, 2]] - pos[1]
        V[5] = pos[0] - pos[1]
        V[6:] = pos
        
        return V 
    
    def gen_marker_array(self, ee_poses, frame_id='hand_root'):
        markers = MarkerArray()
        markers.markers = [Marker(pose=toMsg(ee_poses[i]), type=Marker.SPHERE) for i in range(4)]
        for i, finger in enumerate(self.finger2tip_link):
            markers.markers[i].header.frame_id = frame_id
            markers.markers[i].scale.x = markers.markers[i].scale.y = markers.markers[i].scale.z = 0.01
            markers.markers[i].color.r = 1. - 0.2 * i
            markers.markers[i].color.g = 0. + 0.2 * i
            markers.markers[i].color.a = 1.
            markers.markers[i].ns = '/' + finger
        
        return markers
        

if __name__ == '__main__':
    theta = [
        0.46577540414289814, -0.2890512247250011, 0.12271067718673644, 0.22420640578266968,
        0.46021009223337667, -0.2960440464471773, 0.2982590869972696, 1.084437425534356,
        -0.4411578498505645, -0.3160463959402287, 0.1716352750269975, 0.9928483830978961, 
        0.6074569887040825, 0.9399242057906939, 1.1993804950421847, 1.282356382497728]
    from pprint import pprint
    rospy.init_node('test_fk')
    AKDL = AllegroKDL()
    res, poses = AKDL.solve(theta)
    
    pprint([pose.p for pose in poses])
    
    
    # allegro_pos = [[0.088, 0.104, 0.151], 
    # [0.076, 0.043, 0.149],
    # [0.064, -0.089, 0.157],
    # [0.104, 0.045, 0.062]]