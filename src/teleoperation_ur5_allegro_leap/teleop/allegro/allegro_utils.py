import rospy
from geometry_msgs.msg import Pose


finger_allegro_idx = {'Index': 0, 'Middle': 1,
                          'Ring': 2, 'Pinky': None, 'Thumb': 3}

# in this way, setting the above, changes the order of the fingers
allegro_fingers = [item[0] for item in sorted(finger_allegro_idx.items(), key=lambda x: x[1]) if item[1] is not None] #['Index', 'Middle', 'Ring',  'Thumb']
num_allegro_fingers = len(allegro_fingers)

"""
utility function to know the links of each allegro finger
returns a dict with for each finger a list of links composing it
Returns:
    dict -- a dict finger_name -> list_of_links
"""
allegro_finger2linklist = dict(
    [
        (finger, None) if finger_allegro_idx[finger] is None else
        (finger, ['link_' + str(i + j*4) for i in range(num_allegro_fingers)] + [
            'link_' + str(num_allegro_fingers - 1 + j * num_allegro_fingers ) + '_tip'])
            for finger, j in finger_allegro_idx.items()])

common_allegro_poses = dict()


def define_common_pose(pose_name, finger_pose_list):
    assert len(finger_pose_list)==len(allegro_fingers), 'The finger has to be of the same size of the chosen allegro fingers!'
    common_allegro_poses[pose_name] = []
    for pose in finger_pose_list:
        if isinstance(pose, list) and len(pose) == 7: #3 pos + 4 rot
            pp = Pose()
            pp.position.x = pose[0]  
            pp.position.y = pose[1]
            pp.position.z = pose[2]
            pp.orientation.x = pose[3]
            pp.orientation.y = pose[4]
            pp.orientation.z = pose[5]
            pp.orientation.w = pose[6]
            common_allegro_poses[pose_name].append(pp)
        elif isinstance(pose, Pose):
            common_allegro_poses[pose_name].append(pose)
        else:
            common_allegro_poses.pop(pose_name)
            raise ValueError('The poses can be either: a list of 7 floats or a ROS Pose msg!')

    # common_allegro_poses[pose_name] = dict


def init_finger_planner():
        """ 
        This function initialize dicts of the moveit commanders and initial positions
        NB: This functionality will be removed in future versions
        """
        import moveit_commander
        fingers_commander = dict()
        for finger_name, idx in finger_allegro_idx.items():
            if idx is not None:
                group_string = "allegro_finger" + \
                    str(idx)
                fingers_commander[finger_name] = moveit_commander.MoveGroupCommander(group_string)
                rospy.loginfo(
                    "Loading " + finger_name + " as " + \
                    fingers_commander[finger_name].get_name() + \
                    " ----> " + fingers_commander[finger_name].get_pose_reference_frame() + \
                    " frame")


define_common_pose('relax', 
    [
        [0.0935, 0.0873, 0.1425, 0.0015, 0.9597, 0.2028, 0.194],
        [0.1064, 0.0092, 0.1627, -0.0178, 0.902, 0.0393, 0.4294],
        [0.0979, -0.0651, 0.1759, -0.0005, 0.8645, -0.0846, 0.4953],
        [0.1313, 0.0571, 0.0233, 0.9472, -0.08, 0.2823, 0.1284]
        ])

define_common_pose('wide_open', 
    [
        [0.0704, 0.0564,  0.1285, 0.0468, 0.9956, 0.0682, -0.0429],
        [0.0815, -0.0002, 0.139, 0.0009, 0.9809, -0.0008, 0.1944],
        [0.0689, -0.0519, 0.1396, -0.0373, 0.986, -0.0378, 0.1579],
        [0.0687,  0.117, 0.0563, 0.8699, 0.1961, 0.0134, 0.4522]
        ])

define_common_pose('open_hand', 
    [
        [0.06954543209389911, 0.053660867823116146, 0.21038848954878003, 0.0329588976525037,  0.7304416660585185, 0.02949087870748709, 0.68154161400772],
        [0.07226824948700618, -0.0007293607341892596, 0.21414921429330463, 0.004395210568463723, 0.6864784424706534, -0.0034246513303135516, 0.7271288069595305],
        [0.07598697938894726, -0.04234615983772383, 0.20445618871942695, -0.07795570453665707, 0.7622747913151349, 0.026363996080498334, 0.6420007713128768],
        [0.08695126902009306, 0.128668322984535, 0.043301150931619174, -0.7466191234366368, -0.2878509549329711, -0.3373615715110956, -0.4958718406306697]
        ])