from __future__ import print_function, absolute_import, division
import numpy as np
import yaml

from .pca_synergy import PrincipalComponentAnalysis

class SynergyMapper():
    def __init__(self, input_dims, output_dims=None, mapper_model=None, input_lims=None):
        self.input_dims = input_dims
        self.output_dims = output_dims if output_dims is not None else input_dims
        self.mapper_model = mapper_model
        self.input_lims = input_lims if input_lims is not None else [(0,1) for i in range(len(input_dims))]
        self.input_state = dict([(dim, val) for dim, val in zip(input_dims, [0.0] * len(input_dims))])
    
    def fit(self, X_train):
        pc_data = self.mapper_model.fit_transform(X_train)
        if self.mapper_model.n_components > 0:
            self.input_dims = ['PC_{}'.format(i+1) for i in range(self.mapper_model.n_components)]
            self.input_lims = [(float(l_bound), float(u_bound)) for l_bound, u_bound in zip(pc_data.min(axis=0), pc_data.max(axis=0))]
            self.input_state = dict([(dim, val) for dim, val in zip(self.input_dims, [0.0] * len(self.input_dims))])
        
    def synergy_2_real(self, data=None):
        data = np.asarray([[self.input_state[dim] for dim in self.input_dims]]) if data is None else data
        if self.output_dims is None or self.mapper_model is None:
            return data.tolist()
        else:
            return self.mapper_model.inv_transform(data).tolist()
        
    def real_2_synergy(self, data):
        if self.output_dims is None or self.mapper_model is None:
            return data.tolist()
        else:
            return self.mapper_model.transform(data).tolist()
        
    def to_dict(self):
        data_dict = {
            'input_dims' : self.input_dims,
            'output_dims' : self.output_dims,
            'mapper_model' : self.mapper_model.to_dict(),
            'input_lims' : [list(lim) for lim in self.input_lims],
        }
        return data_dict
    
    @classmethod    
    def from_dict(cls, data_dict):
        mapper_model = cls.model_mux(data_dict['mapper_model']['type'])()
        mapper_model.from_dict(data_dict['mapper_model'])
        obj = cls(
        data_dict['input_dims'],
        data_dict['output_dims'],
        mapper_model,
        [tuple(lim) for lim in data_dict['input_lims']]
        )
        # obj.input_state = dict([(dim, val) for dim, val in zip(obj.input_dims, [0.0] * len(obj.input_dims))])
        return obj
        
    def save(self, filename='synergy_mapper.yaml'):
        data_dict = self.to_dict()
        with open(filename, 'w') as ff:
            yaml.safe_dump(data_dict, ff)
    
    @classmethod
    def load(cls, filename):
        with open(filename, 'r') as ff:
            data_dict = yaml.safe_load(ff)
        return cls.from_dict(data_dict)
    
    @staticmethod
    def model_mux(model_type):
        if model_type == PrincipalComponentAnalysis.__name__:
            return PrincipalComponentAnalysis
        else:
            raise ValueError('Unkown Mapper: {}'.format(model_type))
        
    def __repr__(self):
        return 'SynergyMapper(n_input={}, n_output={}, model={})' .format(
            len(self.input_dims), len(self.output_dims), str(self.mapper_model))