#! /usr/bin/env python
from __future__ import division, print_function
import rospy
import os
import signal

from std_msgs.msg import String
# from Tkinter import *
import Tkinter as tk
# from mmmros.msg import Movement, SensorData

    # def __init__(self, master=None):
    #     tk.Frame.__init__(self, master, width=1080, height=720)
    #     self.master = master
    #     self.pack(fill=tk.BOTH)
    #     self.create_widgets()
    #     self.rpy = [0., 0., 0.]
    #     self.xyz = [0.] * 3
    
class EventCatcher(tk.Frame, object):
    
    """
    EventCatcher Class
    This class manages the window and the key binders for the calibration and teleoperation hotkeys
    """

    @classmethod
    def as_standalone(cls, height=720, width=450, title="Event Catcher"):
        root = tk.Tk()
        root.title(title)
        root.geometry("{}x{}".format(height, width))
        win = cls(master=root, size=(height, width))
        # win.pack()
        return win
    
    def __init__(self, master, size=(300, 300)):
        # super(EventCatcher, self).__init__(screenName=screenName, baseName=baseName, className=className, useTk=useTk, sync=sync, use=use)
        tk.Frame.__init__(self, master, width=size[0], height=size[1])
        self.__master = master

        self.menus = dict() 
        self.menus['base'] = tk.Menu()
        self.define_menu('File')
        self.menus['File'].add_separator()
        self.menus['File'].add_command(label='Close', command=self.quit)

        self.actions = dict()
        self.action_attributes = dict()
        self.entries = dict()
        self.frames = dict()
        self.frames['status_frame'] = tk.Frame(self)
        # self.status_frame = Frame(self)
        self.frames['status_frame'].pack(fill=tk.BOTH, padx=5, pady=5, side=tk.TOP)

        self.frames['entries'] = tk.Frame(self)
        self.frames['entries'].pack(fill=tk.BOTH, side=tk.TOP, padx=5, pady=5)
        
        self.frames['description'] = tk.Frame(self)#, background='black')
        self.frames['description'].pack(fill=tk.BOTH, padx=5, pady=5, side=tk.TOP)

        self.status_string = tk.StringVar(self.frames['status_frame'], '\n'*4)
        
        self.status_label = tk.Label(self.frames['status_frame'], textvariable=self.status_string, anchor='center', 
                        font=("Helvetica", 15),
                        borderwidth=1, relief=tk.GROOVE, #relief=RIDGE, 
                        width=15, height=4).pack(fill=tk.X, expand=True, padx=5, pady=5)#.grid(row=10, column=30)

        self.hotkey_text_labels = []
        self.reset_pub = rospy.Publisher('/reset_sim', String, queue_size=1)
        self.__master.protocol("WM_DELETE_WINDOW", self.onCloseWindow)
        self.__master.geometry( str(size[0]) + "x" + str(size[1]))
        self.__master.bind("<KeyPress>", self.keydown)
        self.__master.bind("<KeyRelease>", self.keyup)

        signal.signal(signal.SIGINT, lambda x,y: self.close_app())
        self.__master.config(menu=self.menus['base'])
        self.pack()

    def define_menu(self, menu_name):
        self.menus[menu_name] = tk.Menu(self.menus['base'], tearoff=0)
        self.menus['base'].add_cascade(label=menu_name, menu=self.menus[menu_name])

    def create_entry(self, name, label_text, default_value=''):
        frame = tk.Frame(self.frames['entries'])
        label = tk.Label(frame, text=label_text, width=25, anchor='w')
        entry = tk.Entry(frame)
        entry.insert(tk.END, default_value)
        # entry.bind('Enter', lambda event: self.frames['description'].focus_set())
        self.entries[name] = entry
        frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        label.pack(side=tk.LEFT)
        entry.pack(side=tk.RIGHT, expand=tk.YES, fill=tk.X)

    def add_frame(self, name, father_frame='description', fill=tk.BOTH, side=tk.TOP, padx=5, pady=10):
        if name not in self.frames:
            self.frames[name] = tk.Frame(self.frames[father_frame])
            self.frames[name].pack(fill=fill, side=side, padx=padx, pady=pady)

    def get_entry(self, entry_name):
        return self.entries[entry_name]

    def get_entry_value(self, entry_name, interpret_as=float):
        # print(())
        entry = self.get_entry(entry_name)
        return interpret_as(entry.get())

    def set_status(self, status):
        self.status_string.set(status)

    def set_title(self, title):
        self.__master.title(title)

    def close_app(self):
        self.__master.quit()
        self.__master.update()

    def bind_action(self, button, function, description=None, frame_name='description'):
        self.actions[button] = function
        self.action_attributes[button] = (self.frames[frame_name], description)

    def onCloseWindow(self):
        self.__master.destroy()
    
    def show_keybinders(self):
        for i, action in enumerate(self.actions):
            
            tk.Label(self.action_attributes[action][0], textvariable=tk.StringVar(self, action), 
                anchor='w', font=("Helvetica", 10), 
                borderwidth=3, relief=tk.FLAT,
            ).grid(padx=5, row=i, column=0, sticky=tk.N+tk.W+tk.E, columnspan=2)#.pack(fill=X, expand=True, padx=5, pady=5, side=LEFT)

            tk.Label(self.action_attributes[action][0], textvariable=tk.StringVar(self, '->'), 
                anchor='w', font=("Helvetica", 10), 
                borderwidth=3, relief=tk.FLAT,
            ).grid(padx=2, row=i, column=3, sticky=tk.N+tk.W+tk.E, columnspan=1)

            tk.Label(self.action_attributes[action][0], textvariable=tk.StringVar(self, self.action_attributes[action][1]), 
                anchor='w', font=("Helvetica", 10), 
                borderwidth=3, relief=tk.FLAT,
            ).grid(padx=3, row=i, column=4, sticky=tk.N+tk.W+tk.E, columnspan=2)#.pack(fill=X, expand=True, padx=5, pady=5, side=LEFT)

    def keydown(self, event):
        if event.keysym in self.actions:
            if self.actions[event.keysym][0] is not None: 
                self.actions[event.keysym][0]()
        else:
            print("{} doesn't exist!".format(event.keysym))
    
    def keyup(self, event):
        if event.keysym in self.actions:
            if self.actions[event.keysym][1] is not None: 
                self.actions[event.keysym][1]()
        self.focus()

    def load_simulation(self, filename):
        filepath_msg = String()
        filepath_msg.data = filename
        self.reset_pub.publish(filepath_msg)
    
if __name__ == "__main__":
    
    rospy.init_node('event_tester')
    ec = EventCatcher.as_standalone()
    ec.set_title("Teleop Controller")
    ec.status_string.set('TerapiaTaioco')
    ec.bind_action('a', (lambda : print('you just pressed a'), None), 'abecedario')
    ec.bind_action('b', (lambda : print('you just pressed b'), None), 'bobba')
    ec.bind_action('F1', (lambda : print('you just pressed F1'), None), 'function')
    ec.bind_action('c', (None, (lambda : print('you just unpressed c'))), 'cocco')
    ec.bind_action('space', (None, (lambda : print('you just unpressed space'))), 'space')
    ec.bind_action('BackSpace', (None, (lambda : print('you just unpressed backspace'))), 'backspace')
    ec.show_keybinders()
    ec.mainloop()
    