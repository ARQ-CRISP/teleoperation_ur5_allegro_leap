
from __future__ import print_function, division, absolute_import
import numpy as np
import tkinter as tk
from functools import partial
import actionlib
import sys

from teleoperation_ur5_allegro_leap.demonstrations.synergies import SynergyMapper, prelearnt_synergies_path
from teleoperation_ur5_allegro_leap.demonstrations.trajectory_learn import Trajectory_Collection_Learner, Joint_Trajectory, prelearnt_trajectories_path
from teleoperation_ur5_allegro_leap.demonstrations import Demonstration_Controller_Pipeline
from allegro_hand_kdl.msg import PoseControlAction, PoseControlGoal, PoseControlResult, PoseControlFeedback



class DemonstrationPlayer(tk.Tk, object):
    
    base_pose_txt = 'Base Pose'
    def __init__(self, screenName=None, baseName=None, className='Tk',
                 useTk=1, sync=0, use=None, size=(550, 400), rosmode=True):
        super(DemonstrationPlayer, self).__init__(screenName=screenName, baseName=baseName,
                                                  className=className, useTk=useTk, sync=sync, use=use)
        
        self.geometry( str(size[0]) + "x" + str(size[1]))
        
        self.title('Demonstration Player')
        self.frames = dict()
        self.frames['list_container'] = tk.Frame(self)
        self.frames['selector'] = tk.Frame(self.frames['list_container'])
        self.frames['selected'] = tk.Frame(self, borderwidth = 1, relief=tk.RIDGE)
        self.frames['player'] = tk.Frame(self, borderwidth = 1, relief=tk.RIDGE)
        self.frames['buttons'] = tk.Frame(self, borderwidth = 1)
        self.frames['generate_buttons'] = tk.Frame(self.frames['buttons'], borderwidth = 0)
        self.frames['generate_avg'] = tk.Frame(self.frames['generate_buttons'], borderwidth = 0)
        self.frames['generate_posterior'] = tk.Frame(self.frames['generate_buttons'], borderwidth = 0)
        # tk.Frame(self.frames['selected']).pack(fill=tk.X, side=tk.RIGHT, padx=150, expand=True)
        self.selector = tk.Listbox(self.frames['selector'], selectmode=tk.SINGLE)
        # self.selector.insert(tk.END, self.base_pose_txt)
        
        self.selected_text = tk.StringVar()
        self.selected_text.set('')
        self.selected_traj_type_text = tk.StringVar()
        self.selected_traj_type_text.set('')
        
        self.selector.select_set(0)#activate(0)
        # self.listbox.select_set(0)
        self.selected = tk.Label(self.frames['selected'], textvariable=self.selected_text)
        self.traj_type = tk.Label(self.frames['selected'], textvariable=self.selected_traj_type_text)
        # self.selected.setvar(self.selected_text)
        self.scroller = tk.Scrollbar(self.frames['selector'])
        self.base_button = tk.Button(
            self.frames['buttons'], text ="Reset", command = self.on_reset_button_pressed)
        self.generate_avg_button = tk.Button(
            self.frames['generate_avg'], text =      " Generate Average Trajectory ",
            command = partial(self.initialise_current_trajectory, average=True))
        self.generate_posterior_button = tk.Button(
            self.frames['generate_posterior'], text ="Generate Posterior Trajectory", 
            command = partial(self.initialise_current_trajectory, average=False))
        
        self.traj_player = None 
        self.demonstration_pipeline = None
        self.current_state = None
        self.current_trajectory = Joint_Trajectory() 
        self.rosmode = rosmode
        if rosmode:
            self.__pose_action_client = actionlib.ActionClient('pose_control_action', PoseControlAction)
            self.__pose_action_client.wait_for_server()
            self.__posegoal = PoseControlGoal()
        
        
    def set_demonstration_pipeline(self, pipeline):
        self.demonstration_pipeline = pipeline
        for i in range(len(self.demonstration_pipeline)):
            self.selector.insert(tk.END, 'demonstration_{}'.format(i+1))
        self.selected_text.set(self.selected_option[1])
        self.selector.config(yscrollcommand = self.scroller.set)
        self.selector.bind('<<ListboxSelect>>', self.on_traj_select)
        self.scroller.config(command = self.selector.yview)
        self.selector.pack(fill=tk.X, side=tk.LEFT, padx=5, pady=5, expand=True)
        self.scroller.pack(fill=tk.Y, side=tk.RIGHT, padx=5, pady=5, expand=False)
        self.selected.pack(fill=tk.X, side=tk.LEFT, padx=5, pady=5, expand=False)
        self.traj_type.pack(fill=tk.X, side=tk.RIGHT, padx=5, pady=5, expand=False)
        
        
        self.frames['selector'].pack(fill=tk.X, side=tk.TOP, padx=5, pady=5, expand=False)
        self.frames['list_container'].pack(fill=tk.BOTH, side=tk.TOP, padx=5, pady=0, expand=False)    
        self.frames['selected'].pack(fill=tk.X, side=tk.TOP, padx=15, pady=2, expand=False)
        self.frames['player'].pack(fill=tk.X, side=tk.TOP, padx=15, pady=2, expand=False)
        self.frames['buttons'].pack(fill=tk.X, side=tk.BOTTOM, padx=10, pady=5, expand=True)
        self.frames['generate_avg'].pack(fill=tk.X, side=tk.LEFT, padx=0, pady=0, expand=True)
        self.frames['generate_posterior'].pack(fill=tk.X, side=tk.RIGHT, padx=0, pady=0, expand=True)
        self.frames['generate_buttons'].pack(fill=tk.BOTH, side=tk.BOTTOM, padx=0, pady=5, expand=False)
        
        
        self.base_button.pack(fill=tk.X, side=tk.TOP, padx=5, pady=0, expand=False)
        self.generate_avg_button.pack(fill=tk.BOTH, side=tk.LEFT, padx=5, pady=0, expand=True)
        self.generate_posterior_button.pack(fill=tk.BOTH, side=tk.RIGHT, padx=5, pady=0, expand=True)
        self.add_slider()
        self.current_state = [0.0] * len(self.demonstration_pipeline.mapper.output_dims)
        
        self.initialise_current_trajectory()
    
    @property
    def selected_option(self):
        idx = self.selector.curselection()
        if idx == "" or len(idx) == 0:
            return 0, self.selector.get(0)
        return (idx[0], self.selector.get(idx[0]))
        
    def add_slider(self):
        self.traj_player = tk.Scale(self.frames['player'], from_=0.0, to=1.0,
                       orient=tk.HORIZONTAL, length=400, 
                       tickinterval=0.1, resolution=0.001, command=self.on_slider_move)
        self.traj_player.pack(fill=tk.X, side=tk.TOP, padx=5, pady=2, expand=False)
    
    def on_traj_select(self, event):
        selection = self.selected_option
        # self.demonstration_pipeline.traj_gen.set_model_idx(selection[0]-1)
        if not selection[1] == self.selected_text.get():
            self.reset_slider()
            print('Current trajectory: {}'.format(selection[1]), sep='')
            sys.stdout.flush()
            self.demonstration_pipeline.traj_gen.set_model_idx(selection[0])
            self.selected_text.set(selection[1])
            self.initialise_current_trajectory()
            
    def reset_slider(self):
        if self.traj_player is not None:
                self.traj_player.set(0.0)
    
    def initialise_current_trajectory(self, average=True):
        selection = self.selected_option
        t = np.linspace(0, 1, 1000)[:, None]
        if average:
            traj = self.demonstration_pipeline.generate_gaus_joint_pos(t, joint_bounds=False)
            self.selected_traj_type_text.set(' Avg Trajectory')
        else:
            traj = self.demonstration_pipeline.generate_posterior_joint_pos(t, n_samp=1)[0]
            self.selected_traj_type_text.set('Posterior Sample')
        # print(t.shape, np.asarray(traj).shape)
        self.current_trajectory.fit(np.asarray(traj))
        self.reset_slider()
        
    def on_slider_move(self, x):
        selection = self.selected_option
        self.demonstration_pipeline.traj_gen.model_idx
        # if selection[0] > 0:
        # self.demonstration_pipeline.traj_gen.set_model_idx(selection[0])
        # joint_pos = self.demonstration_pipeline.generate_gaus_joint_pos(t = float(x), joint_bounds=False)[0]
        joint_pos = self.current_trajectory.generate_trajectory_mean_var(float(x), False).ravel().tolist()
        # else:
        #     joint_pos = [.0] * len(self.demonstration_pipeline.mapper.output_dims)
        # print(float(x), sep=' ')
        print(np.asarray(joint_pos).ravel().round(3))
        sys.stdout.flush()
        if self.rosmode:
            self.move_joints(joint_pos)

    def move_joints(self, joint_pos):
        self.__posegoal.joint_pose = joint_pos
        self.__pose_action_client.send_goal(self.__posegoal, feedback_cb=lambda x: 0)
    
    def on_reset_button_pressed(self):
        self.reset_slider()
        self.go_to_base()
    
    def go_to_base(self):
        base_pose = [.0] * len(self.demonstration_pipeline.mapper.output_dims)
        if self.rosmode:
            self.move_joints(base_pose)
        else:
            self.current_state = base_pose
        print('Going to Base Pose...')
    
    def quit(self):
        self.destroy()
        exit(code=0)
        
    
if __name__ == '__main__':
    print('Loading demonstration models...')
    collection = Trajectory_Collection_Learner.load(prelearnt_trajectories_path + '/pca_indGP_trajectory_gen.yaml')
    mapper = SynergyMapper.load(prelearnt_synergies_path + '/pca_synergy_mapper.yaml')
    print('Demonstration models Loaded!')
    pipeline_loaded = Demonstration_Controller_Pipeline(mapper, collection)  
    player = DemonstrationPlayer(rosmode=False)
    player.set_demonstration_pipeline(pipeline_loaded)
    
    # pipeline_loaded.set_model_idx
    player.mainloop()