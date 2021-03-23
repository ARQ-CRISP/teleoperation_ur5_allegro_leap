# Teleoperation for the UR5+Allegro Robot
Leap based teleoperation package for the UR5 + Allegro Robot



## Requirements 

in your catkin_ws/src folder, execute:
'''bash
git clone https://github.com/ARQ-CRISP/allegro_hand_kdl.git
git clone https://github.com/ARQ-CRISP/haptic_glove_ros.git
git clone https://github.com/ARQ-CRISP/allegro_mujoco.git
'''
follow the guide in those.


## Execution 

'''bash
roslaunch teleoperation_ur5_allegro_leap allegro_teleop_mujoco.launch
'''

## Services

The GUI helps to control the teleop software. The changes are implemented as ROS Services. Those are:
Toggle_Tracking
Toggle_Calibration
Toggle_Control
GoTo_ByName
Get_Fingertip_Distance
Update_Finger_Measure