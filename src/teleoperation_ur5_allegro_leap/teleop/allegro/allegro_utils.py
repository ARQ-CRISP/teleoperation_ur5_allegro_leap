import rospy

num_allegro_fingers = 4
allegro_fingers = ['Index', 'Middle', 'Ring',  'Thumb']

finger_allegro_idx = {'Index': 0, 'Middle': 1,
                          'Ring': 2, 'Pinky': None, 'Thumb': 3}


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


def compute_target_state_relax():
    """
    Utility that generates a relax pose for the 4 fingers.
    Useful for testing
    """
    target_values = {
        "f0":{
            "pos": {"x": 0.0935, "y": 0.0873, "z": 0.1425},
            "rot": {"w": 0.0015, "x": 0.9597, "y": 0.2028, "z": 0.194}},
        "f1":{
            "pos": {"x": 0.1064, "y": 0.0092, "z": 0.1627},
            "rot": {"w": -0.0178, "x": 0.902, "y": 0.0393, "z": 0.4294}},
        "f2":{
            "pos": {"x": 0.0979, "y": -0.0651, "z": 0.1759},
            "rot": {"w": -0.0005, "x": 0.8645, "y": -0.0846, "z": 0.4953}},
        "f3":{
            "pos": {"x": 0.1313, "y": 0.0571, "z": 0.0233},
            "rot": {"w": 0.9472, "x": -0.08, "y": 0.2823, "z": 0.1284}}
            }
    return target_values



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