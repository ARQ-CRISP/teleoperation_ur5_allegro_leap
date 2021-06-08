
from __future__ import print_function, absolute_import, division
from .trajectory_learner import Trajectory_Learner
import numpy as np


class Joint_Trajectory(Trajectory_Learner):
    
    def __init__(self):
        super(Joint_Trajectory, self).__init__()
    
    def __len__(self):    
        return self.traj_steps
    
    def fit(self, X_train, t=None):
        self.models = X_train.T
        self.traj_steps, self.n_dims = X_train.shape
    
    def generate_trajectory_mean_var(self, t=None, gen_std=True):
        t = np.arange(0, self.traj_steps)[:, None] if t is None else t
        t = np.asarray([[t]]) if type(t) in [float, int] else t
        t = (t * (self.traj_steps-1)).astype(int) if t.dtype == np.float else t
        return self.models[:, t.ravel()].T

    def generate_posterior_traj(self, t=None, n_samp=1):
        t = np.arange(0, self.traj_steps)[:, None] if t is None else t
        t = np.asarray([[t]]) if type(t) in [float, int] else t
        t = (t * (self.traj_steps-1)).astype(int) if t.dtype == np.float else t
        return [self.models[:, t.ravel()].T] * n_samp

    def to_dict(self):
        model_dict = super(Joint_Trajectory, self).to_dict()
        model_dict['trajectory'] = self.models.tolist()
        model_dict['traj_len'] = self.traj_steps
        model_dict['n_dims'] = self.n_dims
        return model_dict
    
    def __repr__(self):
        rep = super(Joint_Trajectory, self).__repr__()
        rep += '(length={}, n_dim={})'.format(self.traj_steps, self.n_dims)
        return rep
    
    @classmethod
    def from_dict(cls, model_dict):
        obj = cls()
        obj.fit(np.asarray(model_dict['trajectory']).T)
        return obj