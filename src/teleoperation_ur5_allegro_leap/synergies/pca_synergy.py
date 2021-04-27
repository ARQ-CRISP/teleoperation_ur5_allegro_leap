from __future__ import division, print_function
import numpy as np
import yaml

class PrincipalComponentAnalysis():

    def __init__(self, variance_covered=0.95, norm_data=True, rescale_data=False):
        self.__eigen_vecs = None
        self.__eigen_vals = None 
        self.n_components = 0
        self.__covered_covariance = variance_covered
        self.__norm_data = norm_data
        self.__rescale_data = rescale_data
        self.norm_factors = None 
    
    def __normalize(self, data):
        if self.__rescale_data:
            return (data - self.norm_factors[0]) / self.norm_factors[1]
        else:
            return (data - self.norm_factors[0]) 
    
    def __unnormalize(self, data):
        if self.__rescale_data:
            return (data * self.norm_factors[1] + self.norm_factors[0]) 
        else:
            return data + self.norm_factors[0]
    
    def fit(self, data):
        if self.__norm_data:
            self.norm_factors = (data.mean(axis=0), data.std(axis=0) / np.sqrt(data.shape[0]))
            data = self.__normalize(data)
            
        Cov = np.cov(data.T)
        eigen_val, eigen_vec = np.linalg.eig(Cov)
        eig_order = np.flip(np.argsort(eigen_val))
        # eigen_val, eigen_vec = eigen_val[eig_order], eigen_vec[eig_order]
        self.__eigen_vals, self.__eigen_vecs = eigen_val[eig_order], eigen_vec[eig_order]
        self.n_components = int(
            ((self.__eigen_vals/self.__eigen_vals.sum()).cumsum() < self.__covered_covariance).sum() + 1)
        
    def transform(self, data, n_components=None):
        n_components = self.n_components if n_components is None else n_components
        if self.__norm_data:
            return self.__normalize(data).dot(self.__eigen_vecs[:, :n_components])
        else:
            return data.dot(self.__eigen_vecs[:, :n_components])

    def inv_transform(self, PCs):
        n_components = PCs.shape[1]
        if self.__norm_data:
            return self.__unnormalize(PCs.dot(self.__eigen_vecs[:, :n_components].T))
        else:
            return PCs.dot(self.__eigen_vecs[:, :n_components].T)

    def fit_transform(self, data):
        self.fit(data)
        return self.transform(data)

    def reconstruction_error(self, data, n_components=None):
        return ((data - self.inv_transform(self.transform(data, n_components))) ** 2).mean(axis=1)

    def save_model(self, filename='pca_mapping.yaml'):
        # filepath = Path(filename)
        with open(filename, 'w') as f:
            collection = {
                'eig_vecs': self.__eigen_vecs.tolist(),
                'eig_vals': self.__eigen_vals.tolist(),
                'n_components': int(self.n_components),
                'covered_variance': float(self.__covered_covariance)}
            if self.__norm_data and self.norm_factors is not None:
                collection['norm_parameters'] = dict()
                collection['norm_parameters']['mean'] = self.norm_factors[0].tolist()
                if self.__rescale_data:
                    collection['norm_parameters']['std'] = self.norm_factors[1].tolist()
            yaml.dump(collection, f)

    def load_model(self, filename='pca_mapping.yaml'):
        # filepath = Path(filename)
        with open(filename, 'r') as f:
            data = yaml.safe_load(f)
        self.__eigen_vecs = np.asarray(data['eig_vecs'])
        self.__eigen_vals = np.asarray(data['eig_vals'])
        self.n_components = int(data['n_components'])
        self.__covered_covariance = float(data['covered_variance'])
        if 'norm_parameters' in data:
            self.__norm_data = True
            if 'std' in data['norm_parameters'] and data['norm_parameters']['std'] is not None:
                self.__rescale_data = True
                self.norm_factors = np.asarray(
                    data['norm_parameters']['mean']), np.asarray(
                        data['norm_parameters']['std'])
            else:
                self.__rescale_data = False
                self.norm_factors = (np.asarray(
                    data['norm_parameters']['mean']), None)
        else:
            self.__norm_data = False
            self.norm_factors = None
        return self