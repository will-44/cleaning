#!/usr/bin/env bash

# Select install mode
while true
do
    echo "Select profile to install"
    echo "[1] Full x86_64 platform"
    echo "[2] Arm64 component of arm64 platform"
    echo "[3] x86_64 component of arm64 platform"
    echo ""
    echo "Profile to install :"

    read mode

    if [[ "$mode" =~ ^(1|2|3)$ ]]; then
        break
    fi

    echo "Please enter valid value"
done

# updateing apt-get to insure correct pacages installed
sudo apt update

# installing complementary tools
sudo apt install python3 -y
sudo apt install python3-pip -y
sudo apt install git
sudo apt install net-tools

# Installing ROS Noetic
# enable restricted, universe and multiverse repositories
sudo add-apt-repository restricted
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo apt update

# Setup computer to accept software from packages.ros.org
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt install curl -y # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installing ROS noetic
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# Environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install packaging tools
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential -y

# Initialize rosdep
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update

# Importing Sycobot repository
cd ~
git clone --recursive https://Sycodal@dev.azure.com/Sycodal/Sycobot/_git/Sycobot

# Modify Sycobot/rubyrhod_ws/src/ROS-TCP-Endpoint/src/ros_tcp_endpoint/default_server_endpoint.py
sed -i "s:^#!/usr/bin/env python$:#!/usr/bin/env python3:g" ~/Sycobot/rubyrhod_ws/src/ROS-TCP-Endpoint/src/ros_tcp_endpoint/default_server_endpoint.py

# Source REQUIREMENT.bash file
cd ~/Sycobot/rubyrhod_ws
source src/REQUIREMENT.bash

# Build workspace according to the selected profile
if [ $mode -eq 1 ]; then
    catkin profile set x86_64_platform
elif [ $mode -eq 2 ]; then
    catkin profile set arm64_platform_arm_machine
elif [ $mode -eq 3 ]; then
    catkin profile set arm64_platform_x86_machine
else
    catkin profile set default
fi

catkin clean -y
catkin build

echo "source ~/Sycobot/rubyrhod_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc