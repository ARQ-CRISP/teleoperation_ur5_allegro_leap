
import numpy as np
from Tkinter import *
from functools import partial
import actionlib

from allegro_hand_kdl.msg import PoseControlAction, PoseControlGoal, PoseControlResult, PoseControlFeedback

class SynergyController(Tk, object):
    
    def __init__(self, screenName=None, baseName=None, className='Tk', useTk=1, sync=0, use=None, size=(520, 500)):
        super(SynergyController, self).__init__(screenName=screenName, baseName=baseName, className=className, useTk=useTk, sync=sync, use=use)
        self.geometry( str(size[0]) + "x" + str(size[1]))
        self.frames = dict()
        self.sliders = dict()
        self.embedding_dict = None
        self.title('Synergy Viz')
        self.frames['container'] = Frame(self)
        self.canvas = Canvas(self.frames['container'])
        self.scrollbar = Scrollbar(self.frames['container'], command=self.canvas.yview)
        self.frames['sliders'] = Frame(self.canvas)
        self.canvas.pack(side=LEFT, fill=BOTH, expand=True, padx=15)
        self.frames['sliders'].pack(fill=BOTH, side=TOP, padx=5, pady=5, expand=True)
        
        self.frames['sliders'].bind("<Configure>", self.onSliderFrameConfigure            )
        self.canvas.bind("<Configure>", self.onCanvasConfigure)
        
        self.canvas_window = self.canvas.create_window((4, 4), window=self.frames['sliders'], anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        
        self.__pose_action_client = actionlib.ActionClient('pose_control_action', PoseControlAction)
        self.__pose_action_client.wait_for_server()
        self.__posegoal = PoseControlGoal()
        
        

    def onSliderFrameConfigure(self, event):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        
    def onCanvasConfigure(self, event):
        # width is tweaked to account for window borders
        width = event.width - 4
        self.canvas.itemconfigure(self.canvas_window, width=width)
        
    def add_slider(self, name, fun, limits=(0., 1.)):
        frame = Frame(self.frames['sliders'], relief=GROOVE,)
        label_frame = Frame(frame)
        slider_frame = Frame(frame)
        label_frame.pack(fill=BOTH, side=TOP)
        slider_frame.pack(fill=BOTH, side=BOTTOM)
        slider = Scale(slider_frame, from_=round(limits[0], 2), to=round(limits[1], 2),
                       orient=HORIZONTAL, length=400, 
                       tickinterval=0.1, resolution=0.01, command=fun)
        # And a label for it
        label_1 = Label(label_frame, text=name)
        
        # Use the grid geometry manager to put the widgets in the respective position
        frame.pack(fill=X, side=TOP, pady=5, padx=0)
        
        label_1.pack(fill=BOTH, side=LEFT, padx=5, pady=0)
        slider.pack(fill=BOTH, side=TOP, padx=25, pady=0)
        # slider.pack()
        self.sliders[name] = slider
        
        
    def set_embedding_value(self, value, name):
        if self.synergy_mapper is not None:
            old = self.synergy_mapper.input_state[name]
            self.synergy_mapper.input_state[name] = float(value)
            
            if abs(old - float(value)) > 1e-2:
                joint_goals = self.synergy_mapper.synergy_2_real()
                for joint_goal in joint_goals:
                        self.__posegoal.joint_pose = joint_goal
                        self.__pose_action_client.send_goal(self.__posegoal, feedback_cb=lambda x: 0)
                    
                print(name, value)
            
        
    def set_synergy_mapper(self, synergy_mapper):
        self.synergy_mapper = synergy_mapper
        for i, name in enumerate(sorted(self.synergy_mapper.input_dims, key=lambda x: int(x.split('_')[1]))):
            self.add_slider(name, partial(self.set_embedding_value, name=name), limits=synergy_mapper.input_lims[i])
        
        self.scrollbar.pack(side=RIGHT, fill = Y)
        self.frames['container'].pack(fill=BOTH, side=TOP, padx=5, pady=5, expand=True)
        self.__posegoal.joint_pose = [0.0] * len(self.synergy_mapper.output_dims)
        
    def quit(self):
        self.destroy()
        exit()
        