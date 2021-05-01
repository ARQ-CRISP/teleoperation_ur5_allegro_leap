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
from teleoperation_ur5_allegro_leap.demonstrations.synergies import PrincipalComponentAnalysis, SynergyMapper, prelearnt_synergies_path
from teleoperation_ur5_allegro_leap.demonstrations.trajectory_learn import Trajectory_Collection_Learner, prelearnt_trajectories_path
from teleoperation_ur5_allegro_leap.demonstrations import Demonstration_Controller_Pipeline
from teleoperation_ur5_allegro_leap.demonstrations.app import DemonstrationPlayer


import numpy as np
# from Tkinter import *
from functools import partial
import actionlib

from allegro_hand_kdl.msg import PoseControlAction, PoseControlGoal, PoseControlResult, PoseControlFeedback

pkg_path = RosPack().get_path('teleoperation_ur5_allegro_leap')
model_folder = pkg_path + '/src/teleoperation_ur5_allegro_leap/demonstrations/synergies/prelearnt/'
JOINT_NAMES = ['joint_{}'.format(i) for i in range(16)]

collection = Trajectory_Collection_Learner.load(prelearnt_trajectories_path + '/pca_indGP_trajectory_gen.yaml')
mapper = SynergyMapper.load(prelearnt_synergies_path + '/pca_synergy_mapper.yaml')
pipeline_loaded = Demonstration_Controller_Pipeline(mapper, collection)  
        
trajectory = pipeline_loaded.generate_gaus_joint_pos(t = np.linspace(0, 1, 100)[:,None], joint_bounds=False)




posegoal = PoseControlGoal()
if __name__ == '__main__':
    
    rospy.init_node('synergy_player')
    player = DemonstrationPlayer()
    rospy.on_shutdown(player.quit)
    pose_action_client = actionlib.ActionClient('pose_control_action', PoseControlAction)
    pose_action_client.wait_for_server()
    player.set_demonstration_pipeline(pipeline_loaded)
    # posegoal.joint_pose = [0.0] * 16

    # pipeline_loaded.set_model_idx
    player.mainloop()
    # control_dict = dict([(k,v) for k,v in zip(JOINT_NAMES, [0.0] * len(JOINT_NAMES))])
    
    # pca_mapping = PrincipalComponentAnalysis(1.).load_model(model_folder + 'pca_mapping.yaml')
    # joint_map = SynergyMapper(
    #     JOINT_NAMES,
    #     input_lims=[(-np.pi/4., 3. * np.pi/4) for i in range(len(JOINT_NAMES))])
    # pca_synergy_map = SynergyMapper(
    #     input_dims = ['PC_{}'.format(1 + i) for i in range(pca_mapping.n_components)], 
    #     output_dims = JOINT_NAMES, mapper_model=pca_mapping,
    #     input_lims=[(-5, 5) for i in range(pca_mapping.n_components)])
    # pose_action_client.send_goal(posegoal, feedback_cb=lambda x: 0)
    # rospy.sleep(1)
    # i = 0
    # while not rospy.is_shutdown() and i < 100:
    
    #     posegoal.joint_pose = trajectory[i]
    #     pose_action_client.send_goal(posegoal, feedback_cb=lambda x: 0)
    #     i += 1
    #     rospy.sleep(1e-3)
    #     rospy.loginfo('Controller step {}'.format(i))
    # SyC = SynergyController()
    # SyC.set_synergy_mapper(pca_synergy_map)
    print('The end!')
    
    # rospy.on_shutdown()
    # SyC.close()
    # rospy.spin()
    



