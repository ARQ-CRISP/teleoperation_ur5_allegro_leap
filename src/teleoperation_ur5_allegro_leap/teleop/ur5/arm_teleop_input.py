from logging import debug
import rospy
import actionlib

# from visualization_msgs.msg import Marker
from PyKDL import Frame, Rotation, Vector
from tf_conversions import posemath as pm
from moveit_commander import MoveGroupCommander, roscpp_initialize
from moveit_commander.conversions import list_to_pose, pose_to_list
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

from relaxed_ik.msg import EEPoseGoals, JointAngles
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker

from teleoperation_ur5_allegro_leap.teleop.ur5 import WS_Bounds
from teleoperation_ur5_allegro_leap.teleop.ur5 import InteractiveControl
from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix

from teleoperation_ur5_allegro_leap.srv import Arm_Cartesian_Target, Arm_Cartesian_TargetResponse


class Arm_Teleop_Input(object):
    
    absolute_teleop_mode_rosparam = ur5_teleop_prefix + 'teleop_mode_absolute'
    
    goal_marker_topic  = ur5_teleop_prefix + 'target_marker_debug2'
    
    relaxed_ik_pose_goals_topic = '/relaxed_ik/ee_pose_goals'
    relaxed_ik_solutions_topic = '/relaxed_ik/joint_angle_solutions'
    request_topic = ur5_teleop_prefix + 'arm_pose_targets'
    
    def __init__(self, init_pose=None, rotation_bias=None, workspace=None):
        
        if init_pose is None:
            self.init_pose = Frame(
                Rotation.Quaternion(*[-0.707, -0.000, 0.707, -0.000]),
                Vector(*[0.200, 0.358, 1.146]))
        
        elif (isinstance(init_pose, list) or isinstance(init_pose, tuple)) and len(init_pose) == 2:
            self.init_pose = Frame(
                Rotation.Quaternion(*init_pose[1]),
                Vector(*init_pose[0]))
            
        elif isinstance(init_pose, Frame):
            self.init_pose = init_pose
            
        else:
            raise ValueError('init_pose must be either: \
                a tuple/list of lists containing position and quaternion of the initial pose\
                a kdl.Frame  of the initial pose\
                None value corrisponding to the initial value\
                ')
            
        if rotation_bias is None:
            self.rotation_bias = Frame(
                Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0 ,0))
        
        elif isinstance(rotation_bias, list) and len(init_pose) == 1:
            self.init_pose = Frame(
                Rotation.Quaternion(*rotation_bias[0]))
        
        elif isinstance(rotation_bias, Frame):
            self.rotation_bias = rotation_bias
            
        elif isinstance(rotation_bias, Rotation):
            self.rotation_bias = Frame(rotation_bias)
        
        else:
            ValueError('rotation_bias must be either: \
                a tuple/list of lists containing position and quaternion of the initial pose\
                a kdl.Frame  of the initial pose\
                None value corrisponding to the initial value\
                ')
        
        if workspace is None:
            workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/')
            self.workspace = WS_Bounds.from_center_scale(workspace_bounds['center'], workspace_bounds['scale'])
        else:
            self.workspace = workspace
        
        self.debug_mode = False
        # self.rotation_bias = Frame(Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0 ,0))
        # self.init_pose = Frame(
        #     Rotation.Quaternion(*[-0.707, -0.000, 0.707, -0.000]),
        #     Vector(*[0.200, 0.358, 1.146]))
        
        self.posegoal = EEPoseGoals()
        self.posegoal.ee_poses.append(Pose())
        # self.set_absolute_mode_flag(self.get_absolute_mode_flag()) # sets to default value
        
        self.goal_pub = rospy.Publisher(
            self.relaxed_ik_pose_goals_topic, EEPoseGoals, queue_size=5)
        
        self.set_debug_properties()
        
        # self._set_target_service = rospy.Service(self.request_topic, Arm_Cartesian_Target,
        #                                         lambda msg: Arm_Cartesian_TargetResponse( 
        #                                             self.OnPoseRequest(msg.query) if msg.absolute else self.is_tracking))
        
        # self.ur5_target_subscriber = rospy.Subscriber(
        #     self.request_topic, PoseStamped, self.OnPoseRequest, queue_size=10)
        self.setup_service()
    
    def setup_service(self):
        self._set_target_service = rospy.Service(self.request_topic, Arm_Cartesian_Target,
                                                lambda msg: Arm_Cartesian_TargetResponse( 
                                                    self.OnPoseRequest(msg.query) ))
    
    def OnPoseRequest(self, pose_stamped):
        request = pm.fromMsg(pose_stamped.pose)
        
        # if self.get_absolute_mode_flag():
        request = self.pose_to_relative_frame(request)
        
        corrected = self.correct_bias(request)
        if self.debug_mode:
            
            self.send_marker(
                pm.toMsg(corrected), 'hand_root')
            
        # corrected.p *= 0.1    
        self.goto_pose(
            corrected
            )
        
        return True
    
    def send_marker(self, pose, frame='world'):
        self.target_marker.pose = pose
        tgt = PoseStamped(pose=pose)
        tgt.header.frame_id = frame
        self.marker_target_pub.publish(tgt)
    
    def pose_to_world_frame(self, relative_pose):
        world_f = self.init_pose * relative_pose 
        return world_f
    
    def pose_to_relative_frame(self, world_pose):
        
        init_2_target_f = (self.init_pose.Inverse() * world_pose) 
        return init_2_target_f
        
    def correct_bias(self, frame):
        """Corrects the rotation bias of the targets

        Args:
            frame (PyKDL.Frame): Frame of the target

        Returns:
            corrected_frame (PyKDL.Frame): Frame with corrected_bias
        """
        # tgt = Frame(Rotation.Quaternion(0.5, 0.5, 0.5, -0.5)).Inverse() * Frame(bb) * frame * Frame(bb).Inverse()
        return  self.rotation_bias * frame * self.rotation_bias.Inverse()
    
    
    def set_debug_properties(self):
        self.marker_target_pub = rospy.Publisher(self.goal_marker_topic, PoseStamped, queue_size=1)
        self.target_marker = Marker(type=Marker.ARROW)
        # self.target_marker.type = Marker.ARROW
        self.target_marker.scale.x, self.target_marker.scale.y, self.target_marker.scale.z = 0.2, 0.01, 0.01
        self.target_marker.color.b = self.target_marker.color.a = 1.0
        
        
    def goto_pose(self, goal_pose):
        """sends the target point

        Args:
            goal_pose (numpy.array): array containing the position and orientation relative to the initial position
            of the goal.
        """
        
        self.posegoal.header.stamp = rospy.Time.now()
        self.posegoal.ee_poses[0] = pm.toMsg(goal_pose)
        self.goal_pub.publish(self.posegoal)
        
    
class Combined_Arm_Teleop_Input(Arm_Teleop_Input):
    
    def __init__(self, init_pose=None, rotation_bias=None):
        
        # self.tf_buffer = tf2_ros.Buffer(1.)
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.world_pose = None
        super(Combined_Arm_Teleop_Input, self).__init__(init_pose, rotation_bias)
        
        self.interactive_m = InteractiveControl(
            list(init_pose.p), list(init_pose.M.GetQuaternion()), 'Arm_Base_Pose')
        
        # self.__timer_call = rospy.Timer(1/30, self.__get_world_pose)
        
    def setup_service(self):
        self._set_target_service = rospy.Service(self.request_topic, Arm_Cartesian_Target,
                                                lambda msg: Arm_Cartesian_TargetResponse( 
                                                    super(Combined_Arm_Teleop_Input, self).OnPoseRequest(msg.query) \
                                                        if msg.absolute else self.OnPoseRequest(msg.query)))
        
    def to_markers_frame(self, pose):
        self.interactive_m.frame
        
    def OnPoseRequest(self, pose_stamped):
        request = pm.fromMsg(pose_stamped.pose)
        
        target = Frame() 
        target.M = self.interactive_m.frame.M * request.M
        target.p = self.interactive_m.frame.p + request.p
        # self.interactive_m.frame * request
        # if self.get_absolute_mode_flag():
        request = self.pose_to_relative_frame(target)
        corrected = self.correct_bias(request)
        if self.debug_mode:
            
            self.send_marker(
                pm.toMsg(corrected), 'hand_root')
            
        # corrected.p *= 0.1    
        self.goto_pose(
            corrected
            )
        
    def __get_world_pose(self):
        self.world_pose = pm.fromTf(self.tf_buffer.lookup_transform('world', 'world', rospy.Time.now()))
        


if __name__ == '__main__':
    rospy.init_node('UR5_ee_control_marker')
    workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=None)
    
    if workspace_bounds is None:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' No Workspace Bounds set')
        workspace_bounds = {'center': [0.0, 0.4, 1.280], 'scale': [0.2] * 3}
        
        # workspace = WS_Bounds.from_center_scale([0.242, 0.364, 1.270], [0.2] * 3)
        rospy.set_param('ur5_teleop_config/workspace/', workspace_bounds)
    else:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' workspace_config: {}'.format(workspace_bounds))
    
    
    init_pose = [0.200, 0.358, 1.146], [-0.707, -0.000, 0.707, -0.000]
    rotation_bias = [-0.7071067811865475, 0.7071067811865476, 0 ,0]
    workspace = WS_Bounds.from_center_scale(workspace_bounds['center'], workspace_bounds['scale'])
    controller = InteractiveControl(init_pose[0], init_pose[1])
    inputmanager = Arm_Teleop_Input(init_pose, rotation_bias)
    rospy.spin()