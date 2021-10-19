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
from teleoperation_ur5_allegro_leap.srv import Go_To_Base, Go_To_BaseResponse, Go_To_BaseRequest
from teleoperation_ur5_allegro_leap.srv import Toggle_Tracking, Toggle_TrackingResponse

class Arm_Teleop_Input(object):
    
    absolute_teleop_mode_rosparam = ur5_teleop_prefix + 'teleop_mode_absolute'
    
    goal_marker_topic  = ur5_teleop_prefix + 'target_marker_debug2'
    toggle_rotation_lock_srv = ur5_teleop_prefix + 'toggle_rotation_lock'
    
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
        self._lock_orientation = False
        
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
                                                    self.OnPoseRequest(msg.query)))
        
        self._toggle_rotation_lock_service = rospy.Service(self.toggle_rotation_lock_srv, Toggle_Tracking,
                                                lambda update: Toggle_TrackingResponse( 
                                                    self.toggle_orientation_lock() if update.update else self._lock_orientation))
    
    def OnPoseRequest(self, pose_stamped):
        request = pm.fromMsg(pose_stamped.pose)
        
        if self._lock_orientation:
            request.M = Rotation() if self.last_target is None else self.last_target.M
        
        # if self.get_absolute_mode_flag():
        request.p = Vector(*self.workspace.bind(request.p))
        request = self.pose_to_relative_frame(request)
        
        corrected = self.correct_bias(request)
        if self.debug_mode:
            
            self.send_marker(
                pm.toMsg(corrected), 'hand_root')
            
        # corrected.p *= 0.1    
        self.goto_pose(
            corrected
            )
        self.last_target = corrected
        return True
    
    def toggle_orientation_lock(self):
        self._lock_orientation = not self._lock_orientation
        if self._lock_orientation:
            rospy.loginfo('UR Teleop Orientation: UNLOCKED!')
        else:
            rospy.loginfo('UR Teleop Orientation: LOCKED!')
        return self._lock_orientation
    
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
    
    go_to_base_srv = ur5_teleop_prefix + 'go_to_base'
    marker_request_topic = ur5_teleop_prefix + 'move_marker_pose'
    def __init__(self, init_pose=None, rotation_bias=None, workspace=None):
        
        super(Combined_Arm_Teleop_Input, self).__init__(init_pose, rotation_bias, workspace)
        
        self.old_request = Frame()
        self.interactive_m = InteractiveControl(
            list(init_pose.p), list(init_pose.M.GetQuaternion()), 'Arm_Base_Pose')

        # self.interactive_m.frame = init_pose
        # self.interactive_m.server.setPose("ur5_target_bias", pm.toMsg(init_pose))
        # self.interactive_m.server.applyChanges()
        # print(init_pose)
        # HACK to make the marker go behind by 12 cm (REQUIRES better solution)
        pp = Frame()
        pp.p = Vector(*[0.12, 0., 0.])
        pp.M = self.interactive_m.frame.M
        if init_pose is not None:
            self.set_interactive_pose(PoseStamped(pose=pm.toMsg(pp))) #NOw pressing F8 you go to init position
        # END HACK to make the arm go behind by 12 cm (REQUIRES better solution)
        
        
        
    def setup_service(self):
        self._set_target_service = rospy.Service(self.request_topic, Arm_Cartesian_Target,
                                                lambda msg: Arm_Cartesian_TargetResponse( 
                                                    super(Combined_Arm_Teleop_Input, self).OnPoseRequest(msg.query) \
                                                        if msg.absolute else self.OnPoseRequest(msg.query)))
        
        self._toggle_rotation_lock_service = rospy.Service(self.toggle_rotation_lock_srv, Toggle_Tracking,
                                                lambda update: Toggle_TrackingResponse( 
                                                    self.toggle_orientation_lock() if update.update \
                                                        else self._lock_orientation))
        
        self._set_marker_pose_service = rospy.Service(self.marker_request_topic, Arm_Cartesian_Target,
                                                      lambda msg: Arm_Cartesian_TargetResponse(
                                                          self.set_interactive_pose(msg.query)))

        self.__go_to_base = rospy.Service(self.go_to_base_srv, Go_To_Base,
                                                lambda msg: Go_To_BaseResponse(result=self.go_to_base(msg.base)))
        
    
    def go_to_base(self, base):
        target = PoseStamped(pose=pm.toMsg(Frame()))
        try:
            if base == 'init':
                super(Combined_Arm_Teleop_Input, self).OnPoseRequest(target)
                return True
            else:
                
                self.OnPoseRequest(target)
                return True
                
        except Exception as e:
            return False
        
    def to_markers_frame(self, pose):
        self.interactive_m.frame
        
    def OnPoseRequest(self, pose_stamped):
        request = pm.fromMsg(pose_stamped.pose)
        
        target = Frame() 
        if self._lock_orientation:
            target.M = self.interactive_m.frame.M * self.old_request.M
        else:
            target.M = self.interactive_m.frame.M * request.M
            self.old_request = request
        target.p = self.interactive_m.frame.p + request.p
        target.p = Vector(*self.workspace.bind(target.p))
        # print('CHECK', self.interactive_m.frame.p, self.interactive_m.frame.M.GetQuaternion())
        # print(list(target.p), list(target.M.GetQuaternion()))
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
        
    def set_interactive_pose(self, pose_stamped): 
        pose = pm.fromMsg(pose_stamped.pose)
        self.interactive_m.frame.p += pose.p
        # pose.M = self.interactive_m.frame.M
        self.interactive_m.server.setPose("ur5_target_bias", pm.toMsg(self.interactive_m.frame))
        self.interactive_m.server.applyChanges()
        # print(self.interactive_m.int_mark.pose)

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
    
    # [0.200, 0.358, 1.146]
    
    init_pose = [0.330, 0.358, 1.186], [0., 0.707, 0., 0.707]
    rotation_bias = [-0.7071067811865475, 0.7071067811865476, 0 ,0]
    workspace = WS_Bounds.from_center_scale(workspace_bounds['center'], workspace_bounds['scale'])
    controller = InteractiveControl(init_pose[0], init_pose[1])
    inputmanager = Arm_Teleop_Input(init_pose, rotation_bias)
    rospy.spin()