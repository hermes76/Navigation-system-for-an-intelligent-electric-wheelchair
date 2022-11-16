# Navigation system for an electric wheel-chair

The following work works with:
- create model 2
- kinect 360
- ROS (Robot Operating System)
- ubuntu 18.04 LTS

configuration roomba
https://github.com/AutonomyLab/create_robot



MAPPING
steps:

- sudo usermod -a -G dialout $USER
- source ~/create_ws/devel/setup.bash
- roslaunch create_bringup create_2.launch

- roslaunch freenect_launch freenect.launch

- rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/camera/depth/image_raw

- rosrun ewc_slam ewc_kinect_frame

- rosrun ewc_teleop ewc_teleop_key

- roslaunch ewc_slam ewc_slam.launch slam_methods:=gmapping

save map:
- rosrun map_server map_saver -f <"name">

NAVIGATION

sudo apt-get install ros-melodic-husky-navigation ros-melodic-husky-gazebo ros-melodic-husky-viz

steps:

- sudo usermod -a -G dialout $USER
- source ~/create_ws/devel/setup.bash
- roslaunch create_bringup create_2.launch

- roslaunch freenect_launch freenect.launch

- rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/camera/depth/image_raw

- rosrun ewc_slam ewc_kinect_frame

- roslaunch ewc_2dnav move_base_launch_1.launch

