#!/usr/bin/env python
from __future__ import print_function

import rospy

from abc import ABCMeta, abstractmethod
from leap_motion.msg import leapros, Human#, Hand, Finger, Bone


# Class responsible of defining the reading of the leapmotion data topic and providing it to the main app
class Leap_Subscriber(object):
    __metaclass__ = ABCMeta
    def __init__(self, leap_topic='/leap_motion/leap_device', expiration_in_secs=.3, consumption_rate=60):
        
        self.expiration_in_secs = expiration_in_secs
        self.consumption_rate = consumption_rate
        self.max_memory = rospy.Duration(expiration_in_secs)
        self.leap_topic = leap_topic

        self.latest_human_data = [] # queue holding the latest samples in 
        self.subscriber = None

        
        # self.paused = True
        rospy.on_shutdown(self.shutdown)
    
    @property
    def finger_names(self):
        return ["Thumb", "Index", "Middle", "Ring", "Pinky"]
    
    # This method controls the way the message is used to extract information
    def __OnLeapMessageReceived(self, leap_msg):

        # check the time of the call
        now = leap_msg.header.stamp
        self.latest_human_data.append(leap_msg)
        # delete all the old elements (they are in order so as soon we find a good one all the rest will be)
        for human in self.latest_human_data:
            if now - human.header.stamp < self.max_memory:
                break  # if it is not too old stop throwing away messages
            self.latest_human_data.pop(0)

    def node_run(self):
        ####################################################################################################################
        # Begin listening to leap & ROS loop
        rate = rospy.Rate(self.consumption_rate)
        self.listen_leap()
        rospy.loginfo('Leap Listening started!')
        while not rospy.is_shutdown():
            # static_transform()
            self.consume_data()
            rate.sleep()
    
    def listen_leap(self):
        rospy.loginfo('>>> Subscriber Started!')
        self.subscriber = rospy.Subscriber(self.leap_topic, Human, self.__OnLeapMessageReceived, queue_size=1)
        
    
    def pause_leap(self):
        rospy.loginfo('>>> Subscriber Paused!')
        self.subscriber.unregister()
        self.subscriber = None

    def shutdown(self):
        rospy.loginfo('Stop Listening leap data....')
        self.subscriber = None
        rospy.loginfo('Closing node....')
        rospy.sleep(.5)
    
    @abstractmethod
    def consume_data(self):          
        raise NotImplementedError


# base specialization of the subscriber defined to print the position of the palm of the hands
class Leap_topic_printer(Leap_Subscriber):
    def __init__(self, leap_topic='/leap_motion/leap_device', consumption_rate=50, expirancy_in_secs=.2):
        rospy.init_node('leap_subscriber', anonymous=False)
        super(Leap_topic_printer, self).__init__(leap_topic='/leap_motion/leap_device', expirancy_in_secs=expirancy_in_secs)
        
        self.listen_leap()
        rate = rospy.Rate(consumption_rate)
        while not rospy.is_shutdown():
            self.consume_data()
            rate.sleep()
    
    
    def consume_data(self):
        # leap_data = Human()
        # leap_data.nr_of_gestures.real
        rospy.loginfo("Samples in memory: %d"%len(self.latest_human_data))
        if len(self.latest_human_data) > 0:
            now = rospy.Time.now()
            rospy.loginfo("Oldest sample colledted at: %.5f"%(now - self.latest_human_data[0].header.stamp).to_sec())
            leap_data = self.latest_human_data.pop(0)
            rospy.loginfo('%'*80)
            rospy.loginfo("Tracked items at %.2f fps - hands: %d\tfingers: %d\tgestures: %d" % (leap_data.current_frames_per_second, leap_data.nr_of_hands, leap_data.nr_of_fingers, leap_data.nr_of_gestures))
            # rospy.loginfo(leap_data.right_hand.wrist_position)
            rospy.loginfo('')

            for hand_name, hand in (("Right", leap_data.right_hand), ("Left", leap_data.left_hand)):
                # a = Human()
                # a.right_hand.finger_list
                if hand.is_present:
                    palm = hand.palm_center
                    wrist = hand.wrist_position
                    fingers = hand.finger_list
                    rospy.loginfo(hand_name + " Angles - roll: %.3f | pitch: %.3f | yaw: %.3f" % (hand.roll * 180/3.14, hand.pitch* 180/3.14, hand.yaw* 180/3.14))
                    rospy.loginfo(hand_name + ' Palm - x: %.3f, y: %.3f, x: %.3f'% (palm.x, palm.y, palm.z))
                    rospy.loginfo(hand_name + ' Wrist - x: %.3f, y: %.3f, x: %.3f'% (wrist[0], wrist[1], wrist[2]))
                    rospy.loginfo('')
                    for finger in fingers:
                        # finger = Finger()
                        # bone = Bone()
                        # bone.bone_end.position
                        tip = finger.bone_list[-1].bone_end.position 
                        rospy.loginfo(' '.join([hand_name, self.finger_names[finger.type], 'Tip (%.3f, %.3f, %.3f)' %(tip.x, tip.y, tip.z)]))

                    rospy.loginfo('-'*80)
        else:
            rospy.wait_for_message(self.leap_topic, Human) # don't go polling but wait for data ready



# test main node that prints the palm of the hand detected by the leapmotion
if __name__ == "__main__":
    
    
    try:
        ls = Leap_topic_printer(leap_topic='/leap_motion/leap_device', consumption_rate=30, expirancy_in_secs=.2)
    except rospy.ROSInterruptException as e:
        print(e)

    

    rospy.spin()
