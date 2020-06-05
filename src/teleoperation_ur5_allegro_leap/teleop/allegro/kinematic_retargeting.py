#! /usr/bin/env python
import numpy as np
from moveit_commander.conversions import pose_to_list
from scipy import optimize
from allegro_utils import finger_allegro_idx

def get_vectors(positions):
            S1 = positions[[2],:] - positions[[0,1,3],:]
            S2 = np.concatenate([positions[[3],:] - positions[[0,1],:], positions[[0],:] - positions[[1],:]])
            return np.concatenate([S1, S2])

def get_sf(d, beta = 1.4, eta1=1e-4, eta2=3e-2, eps=1e-3):
    
    isS1 = np.arange(0,6).reshape(-1, 1) <= 2
    s = np.zeros(d.shape)
    f = np.zeros(d.shape)

    s[d > eps] = 1
    s[np.logical_and(d <= eps, isS1)] = 200 # S1
    s[np.logical_and(d <= eps, np.logical_not(isS1))] = 400 # S2

    f[d>eps] = beta * d[d>eps]
    f[np.logical_and(d<=eps, isS1)] = eta1 # S1
    f[np.logical_and(d<=eps, np.logical_not(isS1))] = eta2 # S2
    
    return s, f

def cost(leap_positions, allegro_positions, beta=1.6, eta=[1e-4, 3e-2], eps=1e-3):

    rh = get_vectors(leap_positions.reshape(4,3))
    ra = get_vectors(allegro_positions.reshape(4,3))
    norm_lp = leap_positions.reshape(4,3) / np.linalg.norm(leap_positions.reshape(4,3), axis=1, ord=2, keepdims=True)
    m = np.linalg.norm(allegro_positions.reshape(4,3), axis=1, ord=2, keepdims=True)
    norm_ap = allegro_positions.reshape(4,3) / m
    d = np.linalg.norm(rh,ord=2, axis=1, keepdims=True)
    rh_ = rh / d
    s, f = get_sf(d, beta, eta[0], eta[1], eps)
    return ((np.linalg.norm(ra - f*rh_, axis=1, ord=2, keepdims=True)*s).sum())/2. + .2 * np.sum(m)# + .01 * allegro_positions.reshape(4,3)[:,0].sum()

def kinematic_retargeting_pose_targets(allegro_state, leap_state):
        leap_poses = np.array([finger.ee_position + finger.ee_orientation  for finger_name, finger in allegro_state.fingers.items()])
        allegro_poses = np.array([finger.pose[-1, :] for finger_name, finger in leap_state.items()])
        
        min_fun = lambda x: cost(leap_poses[:,:3].reshape(-1, 1).squeeze(), x)
        bb=np.array([[0, .2],[-.1, .1],[0., .4]]*4)
        # bounds = optimize.Bounds(bb[:,0], bb[:,1])
        x0 = allegro_poses[:,:3].reshape(-1,1).squeeze()
        res = optimize.least_squares(min_fun, x0, xtol=1e-3, ftol=1e-5
                                    , bounds=bb.T.tolist()
                                    )
        # res = optimize.minimize(min_fun, x0, method='SLSQP', options={'ftol': 1e-3, 'disp': False}, bounds=bounds)
        res_positions = res.x.reshape(4,3)
        # print(res_positions.shape, leap_poses[:, 3:].shape)
        res_poses = np.concatenate([res_positions, leap_poses[:, 3:]], axis=1)#.reshape(-1, 1).squeeze().tolist()
        i = 0
        kinematic_targets = dict()
        for finger_name in finger_allegro_idx.keys(): 
            if finger_name is not 'Pinky':
                print(finger_name)
                self.allegro_state[finger_name].ee_position = res_positions[i,:]
                self.allegro_state[finger_name].ee_orientation = np.array([0., 0., 0., 1.])#leap_poses[i,:]
                # kinematic_targets.update(self.allegro_state[finger_name].to_target_dict(True))
                i+=1