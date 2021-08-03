from __future__ import print_function, absolute_import, division
from abc import ABCMeta, abstractmethod
from collections import namedtuple
from copy import deepcopy

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import rospy
import actionlib
import numpy as np

def sigmoid(x): return 1 / (1 + np.exp(-x))
# def velocity_fun(x): return norm(0.5,0.04).pdf(x)/10


class JointMovementManager(object):
    __metaclass__ = ABCMeta
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    ur_state = namedtuple('UR_State', ('position', 'velocity', 'effort'))
    
    time_to_target = 1 / 30.
    update_time = 1 / 15.
    
    
    @classmethod
    def generate_manager(cls, init_state, sim=True):
        if sim:
            cls = [c for c in cls.__subclasses__() if c.__name__.lower().startswith('sim')][0]
        else:
            cls = [c for c in cls.__subclasses__() if c.__name__.lower().startswith('real')][0]
            
        return cls(init_state)
    
    def  __init__(self, init_state=[0., 0., 0., 0., 0., 0.]):
        self.stopped = False
        self.N = 5
        self.last_j_state_target = init_state
        self.last_j_vel = [0.0] * len(init_state)
        self.last_j_trajectory = JointTrajectory(joint_names=self.jnames)
        self.current_j_state = self.ur_state(init_state, [0.0] * len(init_state), [0.0] * len(init_state))
        
    @abstractmethod    
    def define_trajectory(self, js_postion, j_velocity=None, duration=None):
        raise NotImplementedError
    
    @abstractmethod
    def go_to(self, target):
        raise NotImplementedError

    @abstractmethod
    def terminate(self):
        raise NotImplementedError
    
    @abstractmethod
    def emergency_stop(self):
        raise NotImplementedError

    def generate_movement(self, j_target): 
        old_jangles = self.last_j_state_target
        old_jangles2 = np.asarray((self.current_j_state.position))
        dist = np.absolute((np.asarray(j_target) - np.asarray(old_jangles2)))
        T = rospy.Duration(dist.max()/(np.pi/20.))
        velocity = ((np.asarray(j_target) - np.asarray(old_jangles2)) / T.to_sec()).tolist()
        if np.any(dist > 1e-3):
            target = self.define_trajectory(j_target, velocity, T)
            self.go_to(target)
            self.last_j_state_target = j_target
            self.last_j_vel = velocity
            
    def listen_j_state(self, msg):
        ur_joint_idx = [msg.name.index(j) for j in self.jnames]
        # ur_joint_idx = [idx for idx, name in enumerate(msg.name) if name in self.jnames]
        pos = [msg.position[idx] for idx in ur_joint_idx]
        velocity = [msg.velocity[idx] for idx in ur_joint_idx] if len(msg.velocity) > 0 else [0.0] * len(pos)
        effort = [msg.effort[idx] for idx in ur_joint_idx] if len(msg.effort) > 0 else [0.0] * len(pos)
        # rospy.sleep(1/50)
        self.current_j_state = self.ur_state(pos, velocity, effort) 
        
    
    def interp_traj(self, j_target, max_vel, max_duration, N=19):
        duration = max_duration.to_sec() if isinstance(max_duration, rospy.Duration) else max_duration
        old_jangles = np.asarray(self.last_j_state_target)
        old_jangles2 = np.asarray((self.current_j_state.position))
        # print(self.current_j_state)
        #TODO check what happens using the current state that you listen to
        js_position = np.asarray(j_target)
        
        # self.jangles = JointState(name=self.jnames, position=next_pos)
        dist = np.absolute((js_position - old_jangles))
        dist2 = np.absolute((js_position - old_jangles2))
        T = dist2.max()/(np.pi/((N+1)*1.5e0))
        # T = max(dist.max()/(np.pi/((N+1)*1e3)), duration)
        # end_velocity = ((np.asarray(js_position) - np.asarray(old_jangles)) / T)
        t_steps = np.linspace(0, T, N)
        
        steps = sigmoid(10*(np.linspace(0, 1, N)-0.5)).reshape(-1,1)
        pos_steps = old_jangles2 * (1-steps) + (steps) * js_position
        # vel_steps = np.zeros((N, 6))
        vel_increments = np.sin(np.pi * steps)
        
        vel_steps = vel_increments * max_vel 
        
        # vel_steps[:, -1] *= 1/3.
        return (t_steps[-2:-1], pos_steps[-2:-1], vel_steps[-2:-1])
        

class SimJointMovementManager(JointMovementManager):
    
    joint_states_topic = '/move_group/fake_controller_joint_states'
    joint_listener_topic = '/joint_states'
    
    def __init__(self, init_state=[0., 0., 0., 0., 0., 0.]):
        super(SimJointMovementManager, self).__init__(init_state)
        rospy.loginfo("[" + rospy.get_name() + "]" + " Connecting to fake controllers ...")
        self.joint_target_pub = rospy.Publisher(self.joint_states_topic, JointState, queue_size=1)
        rospy.loginfo("[" + rospy.get_name() + "]" + " Connected to Sim Robot!")
        self.joint_listener = rospy.Subscriber(self.joint_listener_topic, JointState, self.listen_j_state)
        
    def define_trajectory(self, js_postion, j_velocity=None, duration=None):
        old_jangles = self.last_j_state_target
        # dist = np.absolute((np.asarray(js_postion) - np.asarray(old_jangles)))
        return JointState(name=self.jnames, position=js_postion)
        # self.jstate_buffer.append(deepcopy(self.jangles)) this is in the go_to
        
    def go_to(self, target):
        if not self.stopped:
            target.header.stamp = rospy.Time.now()
                    # self.jangles.header.seq = k
            self.joint_target_pub.publish(target)
            # rospy.sleep(self.time_to_target)
        
    def emergency_stop(self):
        self.stopped = True
        
    def restart(self):
        self.stopped = False

    def terminate(self):
        self.joint_listener.unregister()
        self.joint_target_pub.unregister()
        
class RealJointMovementManager(JointMovementManager):
    real_robot_action_server = '/scaled_pos_joint_traj_controller/follow_joint_trajectory'
    joint_listener_topic = '/joint_states'
    
    def __init__(self, init_state=[0., 0., 0., 0., 0., 0.]):
        self.stopped = False
        super(RealJointMovementManager, self).__init__(init_state)
        self.joint_target_pub = actionlib.SimpleActionClient(self.real_robot_action_server, FollowJointTrajectoryAction)
        rospy.loginfo("[" + rospy.get_name() + "]" + " Waiting for control server...")
        self.joint_listener = rospy.Subscriber(
            self.joint_listener_topic, JointState, self.listen_j_state, queue_size=1)
        self.joint_target_pub.wait_for_server()
        rospy.loginfo("[" + rospy.get_name() + "]" + " Connected to Robot!")
    
    def go_to(self, target):
        if len(target) > 0 and not self.stopped:
            goal = FollowJointTrajectoryGoal(
                trajectory=JointTrajectory(joint_names=self.jnames))
            goal.trajectory.points = target
            # print(target)
            self.joint_target_pub.cancel_goal()
            # self.joint_target_pub.stop_tracking_goal()
            self.joint_target_pub.send_goal(goal)
        
    def terminate(self):
        self.joint_target_pub.cancel_all_goals()
        # self.joint_target_pub.stop_tracking_goal()
        if self.joint_listener is not None:
            self.joint_listener.unregister()
            self.joint_listener = None
        
    def emergency_stop(self):
        self.stopped = True
        self.joint_target_pub.cancel_all_goals()
        if self.joint_listener is not None:
            self.joint_listener.unregister()
            self.joint_listener = None
        
    def restart(self):
        self.stopped = False
        self.joint_listener = rospy.Subscriber(
            self.joint_listener_topic, JointState, self.listen_j_state, queue_size=1)

    def define_trajectory(self, js_postion, j_velocity, duration):
        old_jangles = self.last_j_state_target
        dist = np.absolute((np.asarray(js_postion) - np.asarray(old_jangles)))
        
        if np.any(dist > 1e-4):
            
            # N = 3
            self.N = 11
            viapoints = self.interp_traj(js_postion, j_velocity, duration, self.N)
            
            traj_points = [
                JointTrajectoryPoint(
                        positions=pos.tolist(),
                        velocities=vel.tolist(), 
                        time_from_start=rospy.Duration(t)) for t, pos, vel in zip(*viapoints)]
            return traj_points
        else: return []
    
    
    
    
    
    
if __name__ == '__main__':
    import yaml
    from rospkg.rospack import RosPack  
    rospy.init_node('test')
    relaxed_ik_path = RosPack().get_path('relaxed_ik')
    relaxed_yaml_filename = rospy.get_param('~relaxed_ik_yaml', default='ur5_allegro_info.yaml')
    yaml_file_path = relaxed_ik_path + '/src/' + '/RelaxedIK/Config/info_files/' + relaxed_yaml_filename
    with open(yaml_file_path) as f:
        yaml_file = yaml.load(f)
    manager = JointMovementManager.generate_manager(yaml_file['starting_config'], sim=True)
    while not rospy.is_shutdown():
        print(type(manager), manager.current_j_state, end="\r")
        rospy.sleep(1)