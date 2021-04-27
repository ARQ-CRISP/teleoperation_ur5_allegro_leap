import numpy as np

class SynergyMapper():
    def __init__(self, input_dims, output_dims=None, mapper_model=None, input_lims=None):
        self.input_dims = input_dims
        self.output_dims = output_dims if output_dims is not None else input_dims
        self.mapper_model = mapper_model
        self.input_lims = input_lims if input_lims is not None else [(0,1) for i in range(len(input_dims))]
        self.input_state = dict([(dim, val) for dim, val in zip(input_dims, [0.0] * len(input_dims))])
    
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