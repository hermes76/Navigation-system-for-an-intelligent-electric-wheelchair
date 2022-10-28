# Navigation system for an electric wheel-chair

The following work works with:
- create model 2
- kinect 360
- ROS (Robot Operating System)
- ubuntu 18.04 LTS

tutorial turtlebot3:
https://youtu.be/3hswO5bAIK4

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

- export TURTLEBOT3_MODEL=burger
- rosrun turtlebot3_teleop turtlebot3_teleop_key

- roslaunch ewc_slam ewc_slam.launch slam_methods:=gmapping

save map:
- rosrun map_server map_saver -f <"name">

NAVIGATION
steps:

- sudo usermod -a -G dialout $USER
- source ~/create_ws/devel/setup.bash
- roslaunch create_bringup create_2.launch

- roslaunch freenect_launch freenect.launch

- rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/camera/depth/image_raw

- rosrun ewc_slam ewc_kinect_frame

- export TURTLEBOT3_MODEL=burger
- roslaunch ewc_2dnav ewc_navigation.launch

