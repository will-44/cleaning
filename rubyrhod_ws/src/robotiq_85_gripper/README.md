# robotiq_85_gripper
ROS package for the Robotiq 2F-85 Gripper using RS485 communication (compatible with Gazebo).

Forked from [StanleyInnovation/robotiq_85_gripper](https://github.com/StanleyInnovation/robotiq_85_gripper) and updated for ROS Noetic.

## Instructions
Defaults to 'ttyUSB0' and 115200 baud rate

Single gripper and left gripper (in dual gripper configuration) need to be configured as device 9 using the Robotiq User Interface
Right gripper (in dual gripper configuration) need to be configured as device 9 using the Robotiq User Interface

start with:
roslaunch robotiq_85_bringup robotiq_85.launch run_test:=true
