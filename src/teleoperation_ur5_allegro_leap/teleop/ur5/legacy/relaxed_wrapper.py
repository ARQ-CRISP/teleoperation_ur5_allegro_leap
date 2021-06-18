
import rospy
from sensor_msgs.msg import JointState


from relaxed_ik.msg import EEPoseGoals
from utils import list2ROSPose
import os
from sklearn.externals import joblib

from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_leap_teleop.config import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame, config_file_name

class Relaxed_UR5_Controller():

    def __init__(self, joint_states_topic='joint_states', relaxed_pose_goals_topic='/relaxed_ik/ee_pose_goals', solver_maxiter=11, solver_maxtime=.05):

        self.__joint_states_topic = joint_states_topic
        self.__relaxed_pose_goals_topic = relaxed_pose_goals_topic
        self.__solver_maxiter = solver_maxiter
        self.__solver_maxtime = solver_maxtime

        self.__relaxedIK = RelaxedIK.init_from_config(config_file_name)
        self.__js_pub = rospy.Publisher(
            self.__joint_states_topic, JointState, queue_size=5)
        self.__ee_pose_goals_pub = rospy.Publisher(
            self.__relaxed_pose_goals_topic, EEPoseGoals, queue_size=3)

    def __init_relaxed_vars(self):
        from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
        from rospkg import RosPack
        dirname = RosPack().get_path('relaxed_ik')
        path = os.path.join(dirname, '/src/RelaxedIK/Config/{}'.format(config_name))
        file_name = open(path, 'r')
        config_data = joblib.load(file_name)

        robot_name = config_data[0]
        collision_nn = config_data[1]
        init_state = config_data[2]
        full_joint_lists = config_data[3]
        fixed_ee_joints = config_data[4]
        joint_order = config_data[5]
        urdf_path = config_data[6]
        collision_file = config_data[7]
        RelaxedIK_vars(urdf_path, full_joint_lists, fixed_ee_joints, joint_order,
                       config_file_name=config_name, collision_file=collision_file,
                       init_state=init_state, rotation_mode='relative',  position_mode='relative')

    @property
    def EE_position(self):
        ee_current = self.__relaxedIK.vars.ee_positions
        return ee_current

    def go_to(self, pos, quat):
        hand_pose = list2ROSPose(xyz=pos, orient=quat)
        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.header.seq = self.idx
        now = rospy.Time.now()
        xopt = self.relaxedIK.solve([pos], [quat], max_iter=self.__solver_maxiter, maxtime=self.__solver_maxtime)
        js = joint_state_define(xopt)
        js.header.stamp.secs = now.secs
        js.header.stamp.nsecs = now.nsecs

        ee_pose_goals.ee_poses.append(hand_pose)

        self.__js_pub.publish(js)
        self.__ee_pose_goals_pub.publish(ee_pose_goals)
