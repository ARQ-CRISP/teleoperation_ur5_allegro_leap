# Teleoperation Package for UR5 + Allegro (Maria) robot
Leap based teleoperation package for the UR5 + Allegro Robot



## Requirements 

in your catkin_ws/src folder, execute:
```bash
git clone -b personal_config https://github.com/Raziel90/relaxed_ik.git
git clone https://github.com/ARQ-CRISP/allegro_hand_kdl.git
git clone https://github.com/ARQ-CRISP/kdl_control_tools.git
git clone https://github.com/ARQ-CRISP/arq_ur5.git
git clone https://github.com/ARQ-CRISP/haptic_glove_ros.git
git clone https://github.com/ARQ-CRISP/allegro_mujoco.git
git clone https://github.com/ARQ-CRISP/ur5_allegro_moveit.git
git clone https://github.com/ARQ-CRISP/ros_topic_connector.git
git clone https://github.com/gokhansolak/allegro-hand-ros.git
git clone https://github.com/ros-drivers/leap_motion.git
git clone https://github.com/ARQ-CRISP/optoforce_publisher.git
git clone https://github.com/ARQ-CRISP/arq_ur5.git
```


follow the guide in those.


## Execution 
### Simulated
```bash
roslaunch teleoperation_ur5_allegro_leap ur5_allegro_teleop_mujoco.launch # full system
roslaunch teleoperation_ur5_allegro_leap allegro_teleop_mujoco.launch # allegro teleop only
roslaunch teleoperation_ur5_allegro_leap relaxed_setup.launch # ur5 teleop
```

## Services

The GUI helps to control the teleop software. The changes are implemented as ROS Services. Those are:
```
Toggle_Tracking
Toggle_Calibration
Toggle_Control
GoTo_ByName
Get_Fingertip_Distance
Update_Finger_Measure
```
## Dependencies 

```bash
sudo apt-get install ros-melodic-python-orocos-kdl
sudo apt-get install ros-melodic-kdl-conversions
sudo apt-get install ros-melodic-kdl-parser      
sudo apt install ros-melodic-moveit-commander

pip install -U python-fcl
pip install -U scikit-learn
```