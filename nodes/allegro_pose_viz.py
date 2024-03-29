#! /usr/bin/env python
'''
author: Claudio Coppola
website: www.claudiocoppola.com
email: c.coppola@qmul.ac.uk
last update: 13/04/21
'''
import numpy as np
import rospy
from rospkg.rospack import RosPack
from teleoperation_ur5_allegro_leap.demonstrations.synergies import PrincipalComponentAnalysis, SynergyMapper
from teleoperation_ur5_allegro_leap.demonstrations.app import SynergyController

pkg_path = RosPack().get_path('teleoperation_ur5_allegro_leap')
model_folder = pkg_path + '/src/teleoperation_ur5_allegro_leap/demonstrations/synergies/prelearnt/'
JOINT_NAMES = ['joint_{}'.format(i) for i in range(16)]

  
        
         



if __name__ == '__main__':
    
    rospy.init_node('synergy_viz')
    # control_dict = dict([(k,v) for k,v in zip(JOINT_NAMES, [0.0] * len(JOINT_NAMES))])
    
    pca_mapping = PrincipalComponentAnalysis(1.).load_model(model_folder + 'pca_mapping.yaml')
    joint_map = SynergyMapper(
        JOINT_NAMES,
        input_lims=[(-np.pi/4., 3. * np.pi/4) for i in range(len(JOINT_NAMES))])
    pca_synergy_map = SynergyMapper(
        input_dims = ['PC_{}'.format(1 + i) for i in range(pca_mapping.n_components)], 
        output_dims = JOINT_NAMES, mapper_model=pca_mapping,
        input_lims=[(-5, 5) for i in range(pca_mapping.n_components)])
    
    SyC = SynergyController()
    SyC.set_synergy_mapper(pca_synergy_map)
    
    rospy.on_shutdown(SyC.quit)
    # SyC.close()

    SyC.mainloop()







