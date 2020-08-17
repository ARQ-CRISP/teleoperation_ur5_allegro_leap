SUBJECT='stefi'
OBJSIZE='small'
EXP_MODE='vision'
BAGNAME='experiment_recordings_'${EXP_MODE}'_'${OBJSIZE}'_'${SUBJECT}
rosbag record -O ~/.ros/rosbags/${BAGNAME}.bag /optoforce_wrench_0 /optoforce_wrench_1 /optoforce_wrench_2 /optoforce_wrench_3 /allegroHand_0/desired_forces /allegroHand_0/joint_states /allegroHand_0/torque_cmd /leap_motion/leap_device /leap_motion/leap_filtered /leap_motion/visualization_marker_array /allegro_teleop_debug /tf /tf_static /vibration_command /vibration_state /experiment_state
