import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
import math
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker



def pose2str(pose):
    rpy = tf.transformations.euler_from_quaternion(
        [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z], 'sxyz')
    pos_str = ['position'] + map(lambda x: str(round(x, 3)),
                                 [pose.position.x, pose.position.y, pose.position.z])
    orient_str = ['orientation'] + \
        map(lambda x: str(round(x*180.0/math.pi, 3)), rpy)
    return ' '.join(pos_str + ['\n'] + orient_str)


def generate_pose_marker(marker_pose, type=Marker.SPHERE, ref_frame='world', name_space='debug_marker', RGBA=[1., 0., 0., .7], scale=[.02, .02, .02]):
    """
    Generate a Marker Message from its pose.
    The function as a default interpret that pose in the world frame, draw a red sphere on the 'debug_marker' namespace

    Arguments:
        marker_pose {Pose} -- pose where you want to plot the marker

    Keyword Arguments:
        type  -- type of the marker (obtainable from marker object) (default: {Marker.SPHERE})
        ref_frame {str} -- frame of reference of the provided pose (default: {'world'})
        name_space {str} -- namespace of the marker (default: {'debug_marker'})
        RGBA {list} -- list of values corresponding to the colors (default: {[1., 0., 0., .7]})
        scale {list} -- scale of the object (default: {[.02, .02, .02]})

    Returns:
        Marker -- Marker message completed with the provided arguments in ADD mode.
    """

    marker = Marker(type=type, ns=name_space, action=Marker.ADD)
    marker.header.frame_id = ref_frame
    # print(ref_frame)
    marker.pose = marker_pose
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.color.r = RGBA[0]
    marker.color.g = RGBA[1]
    marker.color.b = RGBA[2]
    marker.color.a = RGBA[3]
    return marker

def STransform2SPose(Stransform):
    # Stransform = TransformStamped()
    Spose = PoseStamped()

    Spose.pose.position.x = Stransform.transform.translation.x
    Spose.pose.position.y = Stransform.transform.translation.y
    Spose.pose.position.z = Stransform.transform.translation.z
    
    Spose.pose.orientation.x = Stransform.transform.rotation.x
    Spose.pose.orientation.y = Stransform.transform.rotation.y
    Spose.pose.orientation.z = Stransform.transform.rotation.z
    Spose.pose.orientation.w = Stransform.transform.rotation.w

    Spose.header = Stransform.header
    return Spose
    
    pass


def ROSPose2list(pose, RPY=False):
    """
    Converts an object of type Pose to a tuple containing position and orientation in list form
    
    Arguments:
        pose {[Pose]} -- [Pose object containing the info of the position and orientation of an object]
    
    Keyword Arguments:
        RPY {bool} -- [if True the output is RPY angles form; if False the output is a Quaternion] (default: {False})
    
    Returns:
        [tuple] -- [tuple containing the xyz position and the orientation of the pose object]
    """
    if not isinstance(pose, Pose):
        raise ValueError('pose must be of type geometry_msg.msg.Pose')
    xyz = [pose.position.x, pose.position.y, pose.position.z]
    orient = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    if RPY:
        orient = tf.transformations.euler_from_quaternion(orient).tolist()
    return (xyz, orient)

def list2ROSPose(xyz=[0., 0., 0.], orient=[0., 0., 0.]):
    """
    converts the xyz, orient format in geometry_msgs.msg Pose for ROS functionalities

    Keyword Arguments:
        xyz {list} -- x,y,z position (default: {[0., 0., 0.]})
        orient {list} -- r,p,y or x,y,z,w orientation (default: {[0., 0., 0.]})

    Raises:
        ValueError: Raises This exception if the input lists have not the right dimensions

    Returns:
        geometry_msgs.msg.Pose -- Returns the converted data from the input in the Pose message class form
    """

    result = Pose()
    if len(xyz) != 3:
        raise ValueError('xyz should be a list of 3 elements!')
    elif len(orient) < 3 or len(orient) > 4:
        raise ValueError(
            'orient should be a list of 3 (rpy) or 4 (quaternion) elements!')
    else:
        result.position.x = xyz[0]
        result.position.y = xyz[1]
        result.position.z = xyz[2]
        if len(orient) == 3:
            orient = tf.transformations.quaternion_from_euler(
                *orient).tolist()
        result.orientation.x = orient[0]
        result.orientation.y = orient[1]
        result.orientation.z = orient[2]
        result.orientation.w = orient[3]

    return result


# tf_pub = tf.TransformBroadcaster()
tf_pub = tf2_ros.TransformBroadcaster()
def generate_tf(xyz=(0, 0, 0), orient=(0, 0, 0), child='common_world', parent='base', tf_time=None):
    # print(child, parent,  map(lambda x: str(round(x*180.0/math.pi, 3)), xyz + orient))
    # print("preparing tf....")
    if tf_time is None:
        tf_time = rospy.Time()
    if len(xyz) != 3:
        raise ValueError('xyz should be a list of 3 elements!')
    elif len(orient) < 3 or len(orient) > 4:
        raise ValueError(
            'orient should be a list of 3 (rpy) or 4 (quaternion) elements!')
    else:
        if len(orient) == 3:
            orient = tf.transformations.quaternion_from_euler(
                orient[0],
                orient[1],
                -orient[2], axes='szxy').tolist()
        # print("sending tf....")
        Stransform = TransformStamped()
        Stransform.header.stamp = tf_time
        Stransform.header.frame_id = parent
        Stransform.child_frame_id = child
        Stransform.transform.translation.x = xyz[0]
        Stransform.transform.translation.y = xyz[1]
        Stransform.transform.translation.z = xyz[2]
        Stransform.transform.rotation.x = orient[0]
        Stransform.transform.rotation.y = orient[1]
        Stransform.transform.rotation.z = orient[2]
        Stransform.transform.rotation.w = orient[3]
        # print(Stransform.transform)
        tf_pub.sendTransform(Stransform)

    # tf_pub.sendTransform((0, 0, 0),
    #                     tf.transformations.quaternion_from_euler(0, -math.pi, -math.pi),
    #                     rospy.Time.now(),
    #                     'leap_hands',
    #                     'common_world',
    #                     )
    # tf_pub.sendTransform((0, 0, 0),
    #                     tf.transformations.quaternion_from_euler(-math.pi, -math.pi/2, 0),
    #                     rospy.Time.now(),
    #                     'leap_hands',
    #                     'world',
    #                     )
