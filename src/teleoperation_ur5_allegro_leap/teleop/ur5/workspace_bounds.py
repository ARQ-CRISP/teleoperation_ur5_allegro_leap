from __future__ import print_function, division, absolute_import
import numpy as np
import PyKDL as kdl
import yaml
from visualization_msgs.msg import Marker


class WS_Bounds():
    
    def __init__(self, start_pt, end_pt):
        
        self.start_pt = np.min([start_pt, end_pt], axis=0)
        self.end_pt = np.max([start_pt, end_pt], axis=0)
        
    def get_center_scale(self):
        center = (self.start_pt + self.end_pt) * 0.5
        scale = (self.end_pt - self.start_pt) * 0.5
        return center.tolist(), scale.tolist()
    
    def set_center_scale(self, center, scale):
        
        start_pt, end_pt = center - scale, center + scale
        
        self.start_pt = np.min([start_pt, end_pt], axis=0)
        self.end_pt = np.max([start_pt, end_pt], axis=0)
    
    @classmethod
    def from_center_scale(cls, center, scale):
        center, scale = np.asarray(list(center)), np.asarray(list(scale))
        return cls(center - scale, center + scale)
    
    @classmethod
    def from_yaml(cls, yaml_path):
        with open(yaml_path, 'r') as ff:
            data = yaml.load(ff, Loader=yaml.FullLoader)
        if 'center' in data['ur5_teleop_config']['workspace'] and 'scale' in data['ur5_teleop_config']['workspace']:
            return cls.from_center_scale(**data['ur5_teleop_config']['workspace'])
        elif 'start' in data['workspace'] and 'end' in data['workspace']:
            return cls(data['workspace']['start'], data['workspace']['end'])
        else:
            raise Exception('Yaml File not correctly formatted!')
    
    def set_center(self, new_center):
        center, scale = self.get_center_scale()
        return WS_Bounds.from_center_scale(new_center, scale)
        
    def set_scale(self, new_scale):
        center, scale = self.get_center_scale()
        return WS_Bounds.from_center_scale(center, new_scale)
            
    def in_bounds(self, x):
        pt = np.array([list(x)]) if isinstance(x, kdl.Vector) else x
        # kd_pt = kdl.Vector(*x)
        return np.all(
            (np.asarray(pt) <= self.end_pt) & (np.asarray(pt) >= self.start_pt),
            axis=1, keepdims=True)
    
    def get_vertices(self):
        
        merge = np.vstack([self.start_pt, self.end_pt])
        val = np.array(np.meshgrid(merge[:, 0], merge[:, 1], merge[:, 2])).reshape(3, -1).T
        return val
        
    def bind(self, x):
        pt = np.asarray([list(x)]) if isinstance(x, kdl.Vector) else x.copy()
        pt = pt if len(pt.shape) >= 2 else pt.reshape(-1, 1)
        
        bigger_idx = pt > self.end_pt
        smaller_idx = pt < self.start_pt
        pt[bigger_idx] = (self.end_pt * np.ones(pt.shape))[bigger_idx]
        pt[smaller_idx] = (self.start_pt * np.ones(pt.shape))[smaller_idx]
            
        return pt.squeeze()
            
    def to_marker(self, color=[1., 0., 0., 0.]):
        m = Marker(type=Marker.CUBE)
        center, scale = self.get_center_scale()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = center
        m.pose.orientation.w = 1.
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color 
        
    def __repr__(self):
        sta = np.array2string(self.start_pt, precision=3)
        end = np.array2string(self.end_pt, precision=3)
        
        return 'start point: ' + sta + '\tend point: ' + end
    
        
        
        
        
        
if __name__ == '__main__':
    workspace = WS_Bounds([-1, -1, -1], [1., 1., 1.])
    # ws2 = WS_Bounds.from_center_scale(*workspace.get_center_scale())
    # print(ws2.get_center_scale(), (ws2.start_pt, ws2.end_pt))
    x = 4 * np.random.random((10,3)) - 2.
    print(25*'-' + 'Set of arrays input case' + '-'*25)
    res = workspace.in_bounds(x)
    bounded = workspace.bind(x)
    print(
        np.array2string(
            np.hstack([x,
                    #  workspace.in_bounds(x).reshape(-1,1).astype(str),
                     bounded]), precision=3))
    print(res)
    print(25*'-' + 'Check Vector input case' + '-'*25)
    xv = kdl.Vector(*x[0,:].tolist())
    res = workspace.in_bounds(xv)
    bounded = workspace.bind(xv)
    print(
        np.array2string(np.asarray(list(xv)), precision=3),
        np.array2string(bounded, precision=3))
    kdl.Vector(*bounded.squeeze())
    
    print(25*'-' + 'Load from YAML case' + '-'*25)
    curr_fol = '/'.join(__file__.split('/')[:-1])
    yml_bnds = WS_Bounds.from_yaml(curr_fol+'/config/ur5_ee_bounds.yaml')
    print('start/end pts: ', yml_bnds.start_pt, yml_bnds.end_pt)
    print('center/scale: ', *map(lambda x: np.asarray(x).round(4), yml_bnds.get_center_scale()))
    
    
    print(25*'-' + 'Load from repr case' + '-'*25)
    print(yml_bnds)
