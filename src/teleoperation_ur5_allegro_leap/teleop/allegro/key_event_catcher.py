#! /usr/bin/env python
from __future__ import division, print_function
import rospy
import os
import signal
from Tkinter import *
# from mmmros.msg import Movement, SensorData





class EventCatcher(Tk, object):
    
    """
    EventCatcher Class
    This class manages the window and the key binders for the calibration and teleoperation hotkeys
    """

    def __init__(self, screenName=None, baseName=None, className='Tk', useTk=1, sync=0, use=None):
        super(EventCatcher, self).__init__(screenName=screenName, baseName=baseName, className=className, useTk=useTk, sync=sync, use=use)

        self.actions = dict()
        self.action_description = dict()
        self.status_frame = Frame(self)
        self.status_frame.pack(fill=BOTH, padx=5, pady=5, side=TOP)

        self.description_frame = Frame(self)#, background='black')
        self.description_frame.pack(fill=BOTH, padx=5, pady=5, side=TOP)
        
        # self.status_string = ''
        # self.status_label = Label(self.description_frame, 
        #                         textvariable=self.status_string, anchor='center', 
        #                         width=15, height=2).pack(fill=BOTH, padx=5, pady=5)

        self.status_string = StringVar(self.status_frame, '')
        self.title('Event Catcher')
        self.status_label = Label(self.status_frame, textvariable=self.status_string, anchor='center', 
                        font=("Helvetica", 15),
                        borderwidth=1, relief=GROOVE, #relief=RIDGE, 
                        width=15, height=2).pack(fill=X, expand=True, padx=5, pady=5)#.grid(row=10, column=30)

        self.hotkey_text_labels = []

        self.protocol("WM_DELETE_WINDOW", self.onCloseWindow)
        self.geometry("300x300")
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)

        signal.signal(signal.SIGINT, self.close_app)

    def set_status(self, status):
        self.status_string.set(status)

    def set_title(self, title):
        self.title(title)

    def close_app(self, sig, frame):
        self.root.quit()
        self.root.update()

    def bind_action(self, button, function, description=None):
        self.actions[button] = function
        self.action_description[button] = description
    def onCloseWindow(self):
        self.destroy()
    
    def show_keybinders(self):
        for i, action in enumerate(self.actions):
            Label(self.description_frame, textvariable=StringVar(self, action), 
                anchor='w', font=("Helvetica", 10), 
                borderwidth=3, relief=FLAT,
            ).grid(padx=5, row=i, column=0, sticky=N+W+E, columnspan=2)#.pack(fill=X, expand=True, padx=5, pady=5, side=LEFT)

            Label(self.description_frame, textvariable=StringVar(self, '->'), 
                anchor='w', font=("Helvetica", 10), 
                borderwidth=3, relief=FLAT,
            ).grid(padx=2, row=i, column=3, sticky=N+W+E, columnspan=1)

            Label(self.description_frame, textvariable=StringVar(self, self.action_description[action]), 
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
    