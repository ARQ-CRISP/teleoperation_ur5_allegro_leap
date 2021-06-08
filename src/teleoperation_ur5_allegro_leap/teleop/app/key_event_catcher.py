#! /usr/bin/env python
from __future__ import division, print_function
import rospy
import os
import signal

from std_msgs.msg import String
from Tkinter import *
# from mmmros.msg import Movement, SensorData


class EventCatcher(Tk, object):
    
    """
    EventCatcher Class
    This class manages the window and the key binders for the calibration and teleoperation hotkeys
    """

    def __init__(self, screenName=None, baseName=None, className='Tk', useTk=1, sync=0, use=None, size=(300, 300)):
        super(EventCatcher, self).__init__(screenName=screenName, baseName=baseName, className=className, useTk=useTk, sync=sync, use=use)


        self.menus = dict() 
        self.menus['base'] = Menu()
        self.define_menu('File')
        self.menus['File'].add_separator()
        self.menus['File'].add_command(label='Close', command=self.quit)

        self.actions = dict()
        self.action_attributes = dict()
        self.entries = dict()
        self.frames = dict()
        self.frames['status_frame'] = Frame(self)
        # self.status_frame = Frame(self)
        self.frames['status_frame'].pack(fill=BOTH, padx=5, pady=5, side=TOP)

        self.frames['entries'] = Frame(self)
        self.frames['entries'].pack(fill=BOTH, side=TOP, padx=5, pady=5)
        
        self.frames['description'] = Frame(self)#, background='black')
        self.frames['description'].pack(fill=BOTH, padx=5, pady=5, side=TOP)

        self.status_string = StringVar(self.frames['status_frame'], '')
        self.title('Event Catcher')
        self.status_label = Label(self.frames['status_frame'], textvariable=self.status_string, anchor='center', 
                        font=("Helvetica", 15),
                        borderwidth=1, relief=GROOVE, #relief=RIDGE, 
                        width=15, height=2).pack(fill=X, expand=True, padx=5, pady=5)#.grid(row=10, column=30)

        self.hotkey_text_labels = []
        self.reset_pub = rospy.Publisher('/reset_sim', String, queue_size=1)
        self.protocol("WM_DELETE_WINDOW", self.onCloseWindow)
        self.geometry( str(size[0]) + "x" + str(size[1]))
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)

        signal.signal(signal.SIGINT, self.close_app)
        self.config(menu=self.menus['base'])

    def define_menu(self, menu_name):
        self.menus[menu_name] = Menu(self.menus['base'], tearoff=0)
        self.menus['base'].add_cascade(label=menu_name, menu=self.menus[menu_name])

    def create_entry(self, name, label_text, default_value=''):
        frame = Frame(self.frames['entries'])
        label = Label(frame, text=label_text, width=25, anchor='w')
        entry = Entry(frame)
        entry.insert(END, default_value)
        # entry.bind('Enter', lambda event: self.frames['description'].focus_set())
        self.entries[name] = entry
        frame.pack(side=TOP, fill=X, padx=5, pady=5)
        label.pack(side=LEFT)
        entry.pack(side=RIGHT, expand=YES, fill=X)

    def add_frame(self, name, father_frame='description', fill=BOTH, side=TOP, padx=5, pady=10):
        if name not in self.frames:
            self.frames[name] = Frame(self.frames[father_frame])
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
        self.title(title)

    def close_app(self):
        self.quit()
        self.update()

    def bind_action(self, button, function, description=None, frame_name='description'):
        self.actions[button] = function
        self.action_attributes[button] = (self.frames[frame_name], description)

    def onCloseWindow(self):
        self.destroy()
    
    def show_keybinders(self):
        for i, action in enumerate(self.actions):
            
            Label(self.action_attributes[action][0], textvariable=StringVar(self, action), 
                anchor='w', font=("Helvetica", 10), 
                borderwidth=3, relief=FLAT,
            ).grid(padx=5, row=i, column=0, sticky=N+W+E, columnspan=2)#.pack(fill=X, expand=True, padx=5, pady=5, side=LEFT)

            Label(self.action_attributes[action][0], textvariable=StringVar(self, '->'), 
                anchor='w', font=("Helvetica", 10), 
                borderwidth=3, relief=FLAT,
            ).grid(padx=2, row=i, column=3, sticky=N+W+E, columnspan=1)

            Label(self.action_attributes[action][0], textvariable=StringVar(self, self.action_attributes[action][1]), 
                anchor='w', font=("Helvetica", 10), 
                borderwidth=3, relief=FLAT,
            ).grid(padx=3, row=i, column=4, sticky=N+W+E, columnspan=2)#.pack(fill=X, expand=True, padx=5, pady=5, side=LEFT)

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
    ec = EventCatcher()
    ec.title("Teleop Controller")
    ec.status_string.set('TerapiaTaioco')
    ec.bind_action('a', (lambda : print('you just pressed a'), None), 'abecedario')
    ec.bind_action('b', (lambda : print('you just pressed b'), None), 'bobba')
    ec.bind_action('F1', (lambda : print('you just pressed F1'), None), 'function')
    ec.bind_action('c', (None, (lambda : print('you just unpressed c'))), 'cocco')
    ec.bind_action('space', (None, (lambda : print('you just unpressed space'))), 'space')
    ec.bind_action('BackSpace', (None, (lambda : print('you just unpressed backspace'))), 'backspace')
    ec.show_keybinders()
    ec.mainloop()
    