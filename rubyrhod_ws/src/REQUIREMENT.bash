#!/usr/bin/env bash
path=$( dirname -- "${BASH_SOURCE[0]}"; )
sudo apt-get install curl -y
# sudo apt-get install libmove-base-msgs-dev -y # maybe
# sudo apt-get install ros-noetic-navigation -y # maybe
sudo apt-get install ros-noetic-mir-robot -y # floue entre si on utilise la version apt ou en submodule
# sudo apt-get install ros-noetic-soem -y #maybe
sudo apt-get install ros-noetic-rqt* -y # can be kept as it
# sudo apt-get install ros-noetic-socketcan-interface -y # maybe
# sudo apt-get install python-yaml -y # maybe
# sudo apt-get install ros-$ROS_DISTRO-realsense2-camera -y # installé en sous-module git -voir à retirer si possible
# sudo apt-get install ros-noetic-realsense2-description -y
# sudo apt-get install ros-noetic-moveit* -y
sudo pip3 install azure.storage.blob
sudo pip3 install azure-iot-device
sudo pip3 install scipy
sudo pip3 install opencv-python
sudo pip3 install open3d
sudo pip3 install alive-progress
sudo pip3 install pyserial
sudo pip3 install pyclustering

# Install k4a SDK from their repository.
# Bit of a hack, see https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1263
# The simple solution by @vinesmsuic does not seem to work. This uses the manual 
# solution by @atinfinity
# Also, https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1190 is part of this solution (needed to non-interactively accept EULA)

if [ $(dpkg --print-architecture) = "amd64" ]
then
    curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb > /tmp/libk4a1.3_1.3.0_amd64.deb
    curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev/libk4a1.3-dev_1.3.0_amd64.deb > /tmp/libk4a1.3-dev_1.3.0_amd64.deb
    curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0/libk4abt1.0_1.0.0_amd64.deb > /tmp/libk4abt1.0_1.0.0_amd64.deb
    curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/libk4abt1.0-dev_1.0.0_amd64.deb > /tmp/libk4abt1.0-dev_1.0.0_amd64.deb
    curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.3.0_amd64.deb > /tmp/k4a-tools_1.3.0_amd64.deb
    echo 'libk4a1.3 libk4a1.3/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
    echo 'libk4abt1.0	libk4abt1.0/accepted-eula-hash	string	03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | debconf-set-selections
    dpkg -i /tmp/libk4a1.3_1.3.0_amd64.deb
    dpkg -i /tmp/libk4a1.3-dev_1.3.0_amd64.deb
    dpkg -i /tmp/libk4abt1.0_1.0.0_amd64.deb
    dpkg -i /tmp/libk4abt1.0-dev_1.0.0_amd64.deb
    sudo apt-get install -y libsoundio1
    dpkg -i /tmp/k4a-tools_1.3.0_amd64.deb
fi

source /opt/ros/noetic/setup.sh
