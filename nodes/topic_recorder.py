#! /usr/bin/env python
from __future__ import print_function, division
import numpy as np
import rospy
from moveit_commander.conversions import transform_to_list
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from teleoperation_ur5_allegro_leap.msg import Experiment
from tf2_ros import Buffer, TransformListener
from kdl_control_tools.msg import WrenchArray
import concurrent.futures
from os import path
import argparse
from pickle import dump #, load
# from tf2_msgs.msg import TFMessage
# from std_msgs.msg import String
# import pandas as pd


experiment_stages = [
    'BEGIN',
    'ONGOING',
    'ENDING'
]


class Topic_Recorder():
    TOPICS = {
        '/optoforce_wrench_0': WrenchStamped,
        '/optoforce_wrench_1': WrenchStamped,
        '/optoforce_wrench_2': WrenchStamped,
        '/optoforce_wrench_3': WrenchStamped,
        '/allegroHand_0/desired_forces': WrenchArray,
        '/allegroHand_0/joint_states': JointState,
        '/allegroHand_0/torque_cmd': JointState
        
    }

    TFS = [
        ['world', 'object'],
        ['world', 'target'],
        ['world', 'hand_root'],
        ['world', 'right_leap_fingers'],
        ['world', 'palm_link'],
        ['hand_root', 'link_3_tip'],
        ['hand_root', 'link_7_tip'],
        ['hand_root', 'link_11_tip'],
        ['hand_root', 'link_15_tip'],
        ['hand_root', 'leap_hands'],
        ['right_leap_fingers', 'right_leap_hand_wrist'],
        ['right_leap_fingers', 'right_leap_hand_center'],
        ['right_leap_fingers', 'right_Thumb_3'],
        ['right_leap_fingers', 'right_Index_3'],
        ['right_leap_fingers', 'right_Middle_3'],
        ['right_leap_fingers', 'right_Ring_3'],
        ['right_leap_fingers', 'right_Pinky_3'],
    ]
    
    def __init__(self, name, tf_buffer):
        self.name = name
        self.listeners = dict()
        self.data_containers = dict()
        self.tf_buffer = tf_buffer
        self.recording_time = [None, None]
        for topic, top_type in self.TOPICS.items():

            self.listeners[topic] = rospy.Subscriber(
                topic, top_type, queue_size=50,
                callback=self.process_message,
                callback_args=(topic, top_type))
            if top_type is WrenchArray:
                array_length = 4
            elif top_type is JointState:
                array_length = 16
            else:
                array_length = 0

            self.data_containers[topic] = self.create_table(
                top_type, array_length)

        # self.data_containers['TFS'] = []

        for _ in self.TFS:
            self.data_containers['TFS'] = [[] for i in range(len(self.TFS))]
            # pd.DataFrame(columns=[
            #     'stamp', 'source', 'dest',
            #     'x', 'y', 'z',
            #     'qx', 'qy', 'qz', 'qw'])]
        self.recording_time[0] = rospy.Time().now()
        

    def record_tfs_parallel(self, time, workers=4):
        def store_lookup(index):
            target, source = self.TFS[index]
            STransform = self.tf_buffer.lookup_transform(
                    target, source, time, rospy.Duration(.01))
            data_row = [STransform.header.stamp.to_nsec(), source, target] + transform_to_list(STransform.transform)
            self.data_containers['TFS'][index].append(data_row)
        with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
            executor.map(store_lookup, range(len(self.TFS)))
    
    def record_tfs(self, time):
        errors = []
        count_success = 0
        for i, (source, target) in enumerate(self.TFS):
                # self.tf_buffer = Buffer()
            try:
                STransform = self.tf_buffer.lookup_transform(
                    target, source, time, rospy.Duration(.01))
                data_row = [
                    STransform.header.stamp.to_nsec(), source, target] + transform_to_list(STransform.transform)
                self.data_containers['TFS'][i].append(data_row)
                # print(len(self.data_containers['TFS'][i]))
                count_success += 1
            except Exception as e:
                errors.append(e.message)
                pass
                #     rospy.logwarn('Skipping: {source}, {target} at {time}'.format(
                #             source=source, target=target, time=time.to_nsec()))
        # print('Successful TF updates: '+ str(count_success), 'current length : ', len(self.data_containers['TFS'][i]))
        if len(errors) > 0:
            rospy.logwarn('--->\n' + '\n'.join(errors) + '\n')
    def create_table(self, top_type, array_length=0):
        if top_type is WrenchStamped:
            # table = pd.DataFrame(
            #     columns=[[
            #         'stamp', 'seq', 'frame_id',
            #         'force_x', 'force_y', 'force_z',
            #         'torque_x', 'torque_y', 'torque_z', ]]).set_index('stamp')
            table = []

        elif top_type is WrenchArray:
            table = [[] for i in range(array_length)]
            # table = [pd.DataFrame(
            #     columns=[[
            #         'stamp', 'seq', 'frame_id',
            #         'force_x', 'force_y', 'force_z',
            #         'torque_x', 'torque_y', 'torque_z']]).set_index('stamp') for i in range(array_length)]
        elif top_type is JointState:
            # cols = ['stamp', 'seq', 'frame_id']
            # cols += ['joint_' + str(i) for i in range(array_length)]
            # cols += ['velocity_' + str(i) for i in range(int(array_length/3))]
            # table = pd.DataFrame(columns=cols).set_index('stamp')
            table = []
        else:
            table = None
            print(top_type, 'IT WAS NONE!')

        return table

    def process_message(self, msg, topic_args):
        topic_name, topic_type = topic_args

        if topic_type is JointState:
            obj = JointState()

            data_row = [msg.header.stamp.to_nsec(), msg.header.seq,
                        msg.header.frame_id]
            data_row += msg.position
            data_row += msg.velocity
            # data_row += msg.effort
            self.data_containers[topic_name].append(data_row)
            # self.data_containers[topic_name] = self.data_containers[topic_name].append(
            #     dict(zip(self.data_containers[topic_name].columns, data_row)),
            #     ignore_index=True
            # )

        elif topic_type is WrenchStamped:
            data_row = [msg.header.stamp.to_nsec(), msg.header.seq,
                        msg.header.frame_id]
            data_row += [msg.wrench.force.x,
                         msg.wrench.force.y, msg.wrench.force.z]
            data_row += [msg.wrench.torque.x,
                         msg.wrench.torque.y, msg.wrench.torque.z]
            self.data_containers[topic_name].append(data_row)
        #     self.data_containers[topic_name] = self.data_containers[topic_name].append(
        #         dict(zip(self.data_containers[topic_name].columns, data_row)),
        #         ignore_index=True
        #     )

        elif topic_type is WrenchArray:
            for i in range(len(msg.wrenches)):
                data_row = [msg.header.stamp.to_nsec(), msg.header.seq,
                            msg.header.frame_id]
                data_row += [msg.wrenches[i].force.x,
                             msg.wrenches[i].force.y, msg.wrenches[i].force.z]
                data_row += [msg.wrenches[i].torque.x,
                             msg.wrenches[i].torque.y, msg.wrenches[i].torque.z]
                self.data_containers[topic_name][i].append(data_row)
                # self.data_containers[topic_name][i] = self.data_containers[topic_name][i].append(
                #     dict(
                #         zip(self.data_containers[topic_name][i].columns, data_row)),
                #     ignore_index=True
                # )


class RecorderNode:

    def __init__(self, subj_name='', polling_rate=80, buffer_duration_s=16):
        rate = rospy.Rate(polling_rate)
        self.tf_buffer = Buffer(rospy.Duration(buffer_duration_s))
        self.tf_listener = TransformListener(self.tf_buffer)
        self.experiment_subscriber = rospy.Subscriber(
            'experiment_state', Experiment, callback=self.OnExperimentMessage, queue_size=100)
        self.record = dict()
        self.subj_name = subj_name
        self.last_time = rospy.Time().now()
        self.current_state = None
        rospy.spin()
        

    def onShutdown(self):
        rospy.loginfo('>>> Recorder tool closed!')

    def OnExperimentMessage(self, msg):
        # data = String()
        workers = 8 #len(Topic_Recorder.TFS)
        # STATE = data.data.split('[')[1].split(']')[0]
        # name = data.data.split(']')[1].strip()
        STATE = msg.status.split('[')[1].split(']')[0]
        name = msg.name
        self.current_state = STATE
        self.last_time = msg.header.stamp
        # print(STATE)
        if STATE == experiment_stages[0] or name not in self.record:
            self.record[name] = Topic_Recorder(name, self.tf_buffer)
            print('structures ready!')

        elif STATE == experiment_stages[1]:
            
            self.record[name].record_tfs(msg.header.stamp)
            

        elif STATE == experiment_stages[2]:
            # self.record.record_tfs_parallel(self.last_time, workers)
            self.record[name].record_tfs(msg.header.stamp)
            self.record[name].recording_time[1] = msg.header.stamp
            for topic, subs in self.record[name].listeners.items():
                subs.unregister()
                print(topic, np.array(self.record[name].data_containers[topic]).shape)
            
            fileidx = 0
            while path.isfile(name + '_{:04d}_'.format(fileidx) +'.pkl'):
                fileidx += 1
            
            print('TFS', np.array(self.record[name].data_containers['TFS']).shape, np.array(self.record[name].data_containers['TFS'][0]).shape)
            for i, (source, dest) in enumerate(self.record[name].TFS):
                print('|->\t', np.array(self.record[name].data_containers['TFS'][i]).shape)

            result = dict()
            result['name'] = name
            result['data'] = self.record[name].data_containers
            
            result['rec_interval'] = [time.to_nsec() for time in self.record[name].recording_time]
            result['tf_comb'] = self.record[name].TFS
            filename = name + '_{:04d}'.format(fileidx) +'.pkl'
            if self.subj_name is not '':
                filename = '_'.join([self.subj_name, filename])
            rospy.loginfo('saving file: ' + filename)
            with open(filename, 'w') as dumpfile:
                dump(result, dumpfile)
            self.record[name] = None
            

experiment_topic = 'experiment_state'
if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        prog='topic_recorder.py', description="records the topic data from the selected topics.")
    parser.add_argument('-n', action='store', dest='subj_name', type=str, default='')

    parse_res = parser.parse_args()
    rospy.init_node('topic_recorder')
    node = RecorderNode(subj_name=parse_res.subj_name)
