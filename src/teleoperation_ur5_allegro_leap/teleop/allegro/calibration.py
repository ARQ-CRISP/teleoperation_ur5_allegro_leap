import rospy



# TODO BAD Solution the file should be refactored possibly in a readable format
def listoffingers_to_dict(lof):
    fnames = ['f0', 'f2', 'f3', 'f1']
    fingers = dict()
    for i, finger in enumerate(lof):
        # print(i, fnames[i], finger)
        fingers[fnames[i]] = {'pos': {'x': finger[0][0], 'y': finger[0][1], 'z': finger[0][2]},
                              'rot':  {'x': finger[1][0], 'y': finger[1][1], 'z': finger[1][2], 'w': finger[1][3]}}
    return fingers


def set_calibration_pose_param(poses, pose_param):
    # TODO: Other Bad solution, requires better design
    poses['open_hand'] = poses['relax']
    del poses['relax']

    known_poses = dict([(pp['name'], int(p_index[1]))
                        for p_index, pp in rospy.get_param(pose_param).items()])
    indices = []
    pose_idx = len(known_poses)
    for name, pose in poses.items():
        # print(name)
        if name in known_poses.keys():
            indices.append((name, known_poses[name]))
        else:
            indices.append((name, pose_idx))
            pose_idx += 1

    max_idx = get_max_pose_index(pose_param)
    indices = dict(indices)
    # indices = [int(key[1]) if value in poses.keys() else 0 for (key, value) in rospy.get_param(pose_param).items()]

    for i, (key, pose) in enumerate(poses.items()):

        rospy.loginfo("{:d}, {:s}, {:s} ".format(
            i, key, pose_param + "p{}/".format(indices[key]) + "name"))
        rospy.set_param(pose_param + "p{}/".format(indices[key]) + "name", key)
        rospy.set_param(
            pose_param + "p{}/".format(indices[key]) + "state", pose)


def get_max_pose_index(pose_param='allegro_hand_kdl/poses/cartesian_poses'):
    pose_idxs = rospy.get_param(pose_param).keys()
    return max([int(pose_idx[-1]) for pose_idx in pose_idxs])
    # rospy.set_param(pose_param + "p{}/".format(2) + "state", pose)