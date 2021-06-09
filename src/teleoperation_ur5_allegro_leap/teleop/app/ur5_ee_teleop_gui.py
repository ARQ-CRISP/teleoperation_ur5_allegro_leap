import Tkinter as tk
import numpy as np
from tf.transformations import quaternion_from_euler

class UR5_EE_GUI(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master, width=1080, height=720)
        self.master = master
        self.pack(fill=tk.BOTH)
        self.create_widgets()
        self.rpy = [0., 0., 0.]
        self.xyz = [0.] * 3
    
    @classmethod
    def as_standalone(cls, height=720, width=450):
        root = tk.Tk()
        root.title("End Effector control")
        root.geometry("{}x{}".format(height, width))
        gui = cls(master=root)
        return gui
    
    def create_widgets(self):
        top_frame = tk.Frame(self)
        top_frame.pack(side=tk.TOP, fill=tk.X, pady=5)

        # source_name_frame = tk.Frame(top_frame)
        # source_name_frame.pack(side=tk.LEFT, fill=tk.X)

        # target_name_frame = tk.Frame(top_frame)
        # target_name_frame.pack(side=tk.RIGHT, fill=tk.X)

        separator_frame = tk.Frame(top_frame, height=10, bd=1)
        separator_frame.pack(fill=tk.NONE, padx=5, pady=5)

        # source = tk.StringVar(self, value='world')
        # target = tk.StringVar(self, value='hand_root')

        # self.source_label = tk.Label(source_name_frame, text="source", fg="black")
        # self.source_label.pack(side=tk.LEFT, padx=5, pady=5)
        # self.target_label = tk.Label(target_name_frame, text="target", fg="black")
        # self.target_label.pack(side=tk.LEFT, padx=5, pady=5)

        # self.source = tk.Entry(source_name_frame, textvariable=source)#, width=10)
        # self.source.pack(side=tk.LEFT,fill=tk.NONE, expand=tk.FALSE)#, width=10)
        # self.target = tk.Entry(target_name_frame, textvariable=target)
        # self.target.pack(side=tk.LEFT, fill=tk.NONE, expand=tk.FALSE)

        vspace = tk.Frame(self, height=2, bd=1)
        vspace.pack(fill=tk.X, padx=5, pady=5)

        # slider_separator = tk.Frame(self, height=2, bd=1, relief=tk.SUNKEN)
        # slider_separator.pack(fill=tk.X, padx=5, pady=5)

        sliders_frame = tk.Frame(self)
        sliders_frame.pack(side=tk.BOTTOM, fill=tk.BOTH)

        slider_frame_Y = tk.Frame(sliders_frame)
        slider_frame_Y.pack(fill=tk.X)
        slider_frame_P = tk.Frame(sliders_frame)
        slider_frame_P.pack(fill=tk.X)
        slider_frame_R = tk.Frame(sliders_frame)
        slider_frame_R.pack(fill=tk.X)
        
        slider_frame_X = tk.Frame(sliders_frame)
        slider_frame_X.pack(fill=tk.X)
        slider_frame_YY = tk.Frame(sliders_frame)
        slider_frame_YY.pack(fill=tk.X)
        slider_frame_Z = tk.Frame(sliders_frame)
        slider_frame_Z.pack(fill=tk.X)

        self.Y_label = tk.Label(slider_frame_Y, text="Yaw", fg="black")
        self.Y_label.pack(side=tk.LEFT, fill=tk.NONE, padx=10)
        self.slider_Y = tk.Scale(slider_frame_Y, from_=-180, to=180, tickinterval=90/3, length=300, orient=tk.HORIZONTAL, resolution=10)
        self.slider_Y.pack(side=tk.RIGHT, fill=tk.X, expand=tk.TRUE, padx=30)

        self.P_label = tk.Label(slider_frame_P, text="Pitch", fg="black")
        self.P_label.pack(side=tk.LEFT, fill=tk.X, padx=10)
        self.slider_P = tk.Scale(slider_frame_P, from_=-180, to=180, tickinterval=90/3, length=300, orient=tk.HORIZONTAL, resolution=10)
        self.slider_P.pack(side=tk.RIGHT, fill=tk.X, expand=tk.TRUE, padx=30)

        self.R_label = tk.Label(slider_frame_R, text="Roll", fg="black")
        self.R_label.pack(side=tk.LEFT, fill=tk.X, padx=10)
        self.slider_R = tk.Scale(slider_frame_R, from_=-180, to=180, tickinterval=90/3, length=300, orient=tk.HORIZONTAL, resolution=10)
        self.slider_R.pack(side=tk.RIGHT, fill=tk.X, expand=tk.TRUE, padx=30)
        
        
        
        self.X_label = tk.Label(slider_frame_X, text="X", fg="black")
        self.X_label.pack(side=tk.LEFT, fill=tk.X, padx=10)
        self.slider_X = tk.Scale(slider_frame_X, from_=-0.2, to=0.2, tickinterval=1e-1, length=300, orient=tk.HORIZONTAL, resolution=1e-3)
        self.slider_X.pack(side=tk.RIGHT, fill=tk.X, expand=tk.TRUE, padx=30)
        
        self.YY_label = tk.Label(slider_frame_YY, text="Y", fg="black")
        self.YY_label.pack(side=tk.LEFT, fill=tk.NONE, padx=10)
        self.slider_YY = tk.Scale(slider_frame_YY, from_=-0.2, to=0.2, tickinterval=1e-1, length=300, orient=tk.HORIZONTAL, resolution=1e-3)
        self.slider_YY.pack(side=tk.RIGHT, fill=tk.X, expand=tk.TRUE, padx=30)

        self.Z_label = tk.Label(slider_frame_Z, text="Z", fg="black")
        self.Z_label.pack(side=tk.LEFT, fill=tk.X, padx=10)
        self.slider_Z = tk.Scale(slider_frame_Z, from_=-0.2, to=0.2, tickinterval=1e-1, length=300, orient=tk.HORIZONTAL, resolution=1e-3)
        self.slider_Z.pack(side=tk.RIGHT, fill=tk.X, expand=tk.TRUE, padx=30)

        # savebutton_frame = tk.Frame(self)
        # savebutton_frame.pack(side=tk.BOTTOM, fill=tk.NONE)

        # filename_var = tk.StringVar(self, value='leap_hands_transform.yaml')

        # self.filename_label = tk.Label(savebutton_frame, text="filename", fg="black")
        # self.filename_label.pack(side=tk.LEFT, fill=tk.X, padx=10)

        # self.filename = tk.Entry(savebutton_frame, textvariable=filename_var, width=40)
        # self.filename.pack(side=tk.LEFT, fill=tk.NONE, expand=tk.FALSE, padx=30)

        # self.save_button = tk.Button(savebutton_frame, text="Save", command=self.onSaveButtonPressed)
        # self.save_button.pack(side=tk.LEFT, pady=30)
        # tk.IntVar()
        # self.quit.pack(side="bottom")

    def value_changed(self):
        return [self.slider_Y.get(), self.slider_P.get(), self.slider_R.get()] != self.rpy or \
            [self.slider_X.get(), self.slider_YY.get(), self.slider_Z.get()] != self.xyz

    def get_YPR(self):
        self.rpy = [self.slider_Y.get(), self.slider_P.get(), self.slider_R.get()]
        return np.asarray(self.rpy)
    
    def get_XYZ(self):
        self.xyz = [self.slider_X.get(), self.slider_YY.get(), self.slider_Z.get()]
        return np.asarray(self.xyz)
    
    def get_quat(self):
        quat = quaternion_from_euler(*(self.get_YPR()*np.pi/180).tolist())
        return quat

    def get_source(self):
        return self.source.get()

    def get_target(self):
        return self.target.get()

    def get_filename(self):
        return self.filename.get()