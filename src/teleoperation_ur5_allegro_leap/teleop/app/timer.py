#! /usr/bin/env python
from __future__ import division, print_function
from threading import Timer, Lock
from time import time
import rospy


class RealTimer():
    def __init__(self, period, callback, oneshot=False):

        self.period = period if type(
            period) is rospy.Duration else rospy.Duration(period)
        self.function = callback
        self.__oneshot = oneshot
        
        self.__stop = False
        self.__scheduling_lock = Lock()

        self.init_time = self._get_time()
        self.init_rostime = rospy.Time.now()
        self.__schedule_timer(self.period, self.init_time,
                              self.init_time, self.init_time + self.period)


    def __schedule_timer(self, new_period, current_real, current_expected, next_expected):
        self.timer = Timer(
            new_period.to_sec(),
            self.__callback,
            kwargs={
                'last_real': current_real,
                'last_expected': current_expected,
                'current_expected': next_expected})
        self.timer.setDaemon(True)
        self.timer.start()

    def shutdown(self):
        with self.__scheduling_lock:
            self.__stop = True
            if self.timer is not None:
                self.timer.cancel() 

    def _get_time(self):
        return rospy.Time(time())

    def __callback(self, last_real, last_expected, current_expected):
        current_ros = rospy.Time.now()
        current_real = self._get_time()
        event = rospy.timer.TimerEvent(
            last_expected, last_real, current_expected, current_real, 
            current_real - last_real)
        new_period, next_expected = self.__run(event, current_ros)
        
        with self.__scheduling_lock:
                if (self.__oneshot == False or self.timer is None) and not self.__stop:
                    self.__schedule_timer(
                        new_period, current_real, current_expected, next_expected)
                    # self.timer.join()

    def __run(self, event, current_ros):
        event.rostime = current_ros
        self.function(event)
        next_expected = event.current_expected + self.period
        new_period = (event.current_expected - event.current_real) + self.period
        return new_period, next_expected



if __name__ == "__main__":
    def callback(event):
        print('Timer called at {:.3f} -> rostime: {:.3f}'.format(
            (event.current_real - init_time).to_sec(), 
            (event.rostime - init_ros_time).to_sec()))

    rospy.init_node('banana')
    timer = RealTimer(1.0, callback, oneshot=False)
    init_time = timer.init_time
    init_ros_time = rospy.Time.now()#timer.init_rostime
    print('START TIMER!')

    rospy.spin()
