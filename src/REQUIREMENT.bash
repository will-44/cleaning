
apt-get install libmove-base-msgs-dev -y
sudo apt-get install ros-noetic-navigation -y
sudo apt-get install ros-noetic-robot-localization -y
sudo apt install ros-noetic-mir-robot -y
sudo apt install ros-noetic-soem -y
sudo apt-get install ros-noetic-rqt* ros-noetic-moveit* ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers ros-noetic-ros-controllers ros-noetic-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-joint-state-publisher -y
sudo apt install ros-noetic-socketcan-interface -y
sudo apt install ros-noetic-costmap-queue -y
sudo apt install ros-noetic-dwb-critics -y
sudo apt install ros-noetic-move-base-msgs -y
sudo apt install ros-noetic-rospy-message-converter -y
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera -y
sudo apt install ros-noetic-realsense2-description
rm -rf robotiq/robotiq_3f*
pip install azure.storage.blob
pip install azure-iot-device
pip install scipy
pip install opencv-python
pip install open3d
pip install alive-progress
pip install numba
source /opt/ros/noetic/setup.sh