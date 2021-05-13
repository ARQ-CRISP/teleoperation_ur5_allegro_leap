from __future__ import division, print_function, absolute_import, unicode_literals
import numpy as np
import yaml
import GPy

from .trajectory_learner import Trajectory_Learner


class IndependentGPTrajectoryLearner(Trajectory_Learner):
    
    def __init__(self, rbf_sigma=1e-2, noise_sigma=1e-2, lenghtscale=1e-2):
        super(IndependentGPTrajectoryLearner, self).__init__()
        self.rbf_sigma = rbf_sigma
        self.noise_sigma = noise_sigma
        self.rbf_l = lenghtscale
        
    def fit(self, X_train, t=None):
        self.n_dims = X_train.shape[1]
        self.traj_steps = X_train.shape[0]
        t = np.arange(0, self.traj_steps)[:, None]
        t = t / X_train.shape[0]
        # self.T = t
        for i in range(self.n_dims):
            self.models.append(
                GPy.models.GPRegression(
                    t, X_train[:, i:i+1], GPy.kern.RBF(1)
                    # , noise_var=self.noise_sigma
                    # , name='Component_{}'.format(i+1)
                    ))
            self.models[-1]['rbf.lengthscale'] = self.rbf_l
            self.models[-1]['rbf.variance'] = self.rbf_sigma
            self.models[-1]['.*Gaussian_noise.variance'] = self.noise_sigma
            
    def generate_trajectory_mean_var(self, t=None, gen_std=True):
        t = np.arange(0, self.traj_steps)[:, None] / self.traj_steps if t is None else t
        t = np.asarray([[t]]) if type(t) in [float, int] else t
        t = t.astype(float) / self.traj_steps if t.dtype == np.int else t
        mean_std = [model.predict(t) for model in self.models]
        mean_traj = np.hstack([mean_std[i][0] for i in range(self.n_dims)])
        if gen_std:
            std_traj = np.hstack([mean_std[i][1] for i in range(self.n_dims)])
            return mean_traj, std_traj
        else:
            return mean_traj
        
    def generate_posterior_traj(self, t=None, n_samp=1):
        t = np.arange(0, self.traj_steps)[:, None] / self.traj_steps if t is None else t
        t = np.asarray([[t]]) if type(t) in [float, int] else t
        t = t.astype(float) / self.traj_steps if t.dtype == np.int else t
        post =  [model.posterior_samples_f(t, full_cov=True, size=n_samp) for model in self.models]
        traj = []
        for i in range(n_samp):
            traj.append(np.hstack([p[:, :, i] for p in post]))
        
        return traj
    
    def to_dict(self):
        model_dict = super(IndependentGPTrajectoryLearner, self).to_dict()
        model_dict.update({
            'model': [m.to_dict() for m in self.models],
            'traj_steps': self.traj_steps,
            'n_dims': self.n_dims})
        
        return model_dict
    
    @classmethod
    def from_dict(cls, dict_model):
        # rbf_l = dict_model['model'][0]['rbf.lengthscale']
        # rbf_sigma = dict_model['model'][0]['rbf.variance']
        # noise_sigma = dict_model['model'][0]['.*Gaussian_noise.variance']
        obj = cls()#(rbf_sigma=rbf_sigma, noise_sigma=noise_sigma, lenghtscale=rbf_l)
        obj.models = [GPy.models.GPRegression.from_dict(submodel) for submodel in dict_model['model']]
        obj.rbf_sigma = obj.models[0]['rbf.lengthscale']
        obj.noise_sigma = obj.models[0]['rbf.variance']
        obj.lenghtscale = obj.models[0]['.*Gaussian_noise.variance']
        obj.n_dims = dict_model['n_dims']
        obj.traj_steps = dict_model['traj_steps']
        return obj

    
    def __repr__(self):
        return 'IndependentGPTrajectoryLearner(n_components={}, noise_sigma={})'.format(
            len(self.models), round(self.noise_sigma, 3))
        