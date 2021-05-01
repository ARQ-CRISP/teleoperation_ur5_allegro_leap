from __future__ import division, absolute_import, print_function
from abc import ABCMeta, abstractmethod
import yaml


class Trajectory_Learner():
    __metaclass__ = ABCMeta
    
    def __init__(self):
        self.models = []
        self.n_dims = 0
        self.traj_steps = 0
        
    @abstractmethod
    def fit(self, X_train, t=None):
        raise NotImplementedError()
    
    @abstractmethod
    def generate_trajectory_mean_var(self, t=None, gen_std=True):
        raise NotImplementedError()

    @abstractmethod
    def generate_posterior_traj(self, t=None, n_samp=1):
        raise NotImplementedError()
    
    def to_dict(self):
        return {'type': self.__class__.__name__}
    
    def save(self, filename='trajectory_model.yaml'):
        save_dict = self.to_dict()
        with open(filename, 'w') as ff:
            yaml.safe_dump(save_dict, ff)
        
    @classmethod   
    def from_dict(cls, model_dict):
        raise NotImplementedError()
    
    @classmethod
    def load(cls, filename):
        with open(filename, 'r') as ff:
            load_dict = yaml.safe_load(ff)
        if not load_dict['type'] == cls.__name__:
            raise ValueError(
                "The imported model is of type {} can't be loaded as {}".format(
                    load_dict['type'], cls.__name__))
            
        return cls.from_dict(load_dict)
    
    def __repr__(self):
        return self.__class__.__name__


