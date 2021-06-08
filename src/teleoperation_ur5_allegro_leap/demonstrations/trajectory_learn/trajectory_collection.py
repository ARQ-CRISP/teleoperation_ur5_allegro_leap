from __future__ import print_function, division, absolute_import

from paramz import model
from .trajectory_learner import Trajectory_Learner
from .independentGP_learner import IndependentGPTrajectoryLearner
from .joint_trajectory import Joint_Trajectory

class Trajectory_Collection_Learner(Trajectory_Learner):
    
    def __init__(self, model_list):
        super(Trajectory_Collection_Learner, self).__init__()
        self.models = model_list
        self.model_idx = 0
        self.n_traj = len(self.models)
        
    def __repr__(self):
        return '\n'.join([str(model) for model in self.models])
    
    def __getitem__(self, key):
        return self.models[key]
    
    def __setitem__(self, key, value):
        self.models[key] = value
        
    def __len__(self):
        return len(self.models)
        
    def set_model_idx(self, idx):
        
        self.model_idx = idx % len(self.models)
        
    def fit(self, X_Train, t=None):
        if not len(self.models) == len(X_Train):
            raise ValueError(
                'X_train is a list of coordinate episodes. expected {}, received{}'.format(len(self.models), len(X_Train))
                )
        for x_train, model in zip(X_Train, self.models):
            model.fit(x_train, t)
    
    def generate_trajectory_mean_var(self, t=None, gen_std=True):
        return self.models[self.model_idx].generate_trajectory_mean_var(t=t, gen_std=gen_std)
    
    def generate_posterior_traj(self, t=None, n_samp=1):
        return self.models[self.model_idx].generate_posterior_traj(t=t, n_samp=n_samp)
    
    def to_dict(self):
        model_dict = super(Trajectory_Collection_Learner, self).to_dict()
        model_dict.update({
            'model': [m.to_dict() for m in self.models],
            'n_dims': self.n_dims,
            'model_idx': self.model_idx})
        return model_dict
    
    @classmethod
    def from_dict(cls, dict_model):
        models = [cls.learner_mux(m['type']).from_dict(m) for m in dict_model['model']]
        obj = cls(models)
        obj.set_model_idx(dict_model['model_idx'])
        return obj

    @staticmethod
    def learner_mux(learner_type):
        if learner_type == IndependentGPTrajectoryLearner.__name__:
            return IndependentGPTrajectoryLearner
        elif learner_type == Joint_Trajectory.__name__:
            return Joint_Trajectory
        elif learner_type == Trajectory_Collection_Learner.__name__:
            return Trajectory_Collection_Learner
        else:
            raise ValueError('Type of model not supported! You may have to update the library!')

