from __future__ import division, print_function, absolute_import
import numpy as np

from .trajectory_learn import Trajectory_Collection_Learner

class Demonstration_Controller_Pipeline():
    
    def __init__(self, mapper, traj_gen):
        self.mapper = mapper
        if type(traj_gen) is list:
            self.traj_gen = Trajectory_Collection_Learner(traj_gen)
        else:
            self.traj_gen = traj_gen
        
    def fit(self, joint_episodes, t=None):
        self.mapper.fit(np.vstack(joint_episodes))
        pc_episodes = [np.asarray(self.mapper.real_2_synergy(episode)) for episode in joint_episodes]
        self.traj_gen.fit(pc_episodes, t)
        
    def generate_gaus_joint_pos(self, t, joint_bounds=True):
        avg_traj, std_traj = self.traj_gen.generate_trajectory_mean_var(t)
        
        if not joint_bounds:
            return self.__convert_to_joints(avg_traj)
        return self.__convert_to_joints(avg_traj, std_traj)
        
    def generate_posterior_joint_pos(self, t, n_samp=1):
        trajectories = self.traj_gen.generate_posterior_traj(t, n_samp)
        return [self.__convert_to_joints(trajectory) for trajectory in trajectories]
    
    def __convert_to_joints(self, avg_traj, std_traj=None):
        avg_joints = self.mapper.synergy_2_real(avg_traj)
        if std_traj is None:
            return avg_joints    
        else:
            u_bound = self.mapper.synergy_2_real(avg_traj + std_traj)
            l_bound = self.mapper.synergy_2_real(avg_traj - std_traj)
            return avg_joints, (l_bound, u_bound)
    
    def __len__(self):
        return len(self.traj_gen)