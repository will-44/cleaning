FROM osrf/ros:noetic-desktop-full

# select bash as default shell
SHELL ["/bin/bash", "-c"]

# install catkin
RUN apt update && apt install python3-catkin-tools python3-wstool -y
RUN source /opt/ros/noetic/setup.bash

# install ros package
RUN apt-get update && apt-get install -y \
    pip -y \
    libmove-base-msgs-dev -y \
    ros-noetic-navigation -y \
    ros-noetic-robot-localization -y \
    ros-noetic-mir-robot -y \
    ros-noetic-soem -y \
    ros-noetic-rqt* ros-noetic-moveit*  \
    ros-noetic-gazebo-ros-control  \
    ros-noetic-joint-state-controller  \
    ros-noetic-effort-controllers  \
    ros-noetic-position-controllers  \
    ros-noetic-ros-controllers  \
    ros-noetic-ros-control  \
    ros-noetic-ros-numpy \
    ros-noetic-joint-state-publisher-gui  \
    ros-noetic-joint-state-publisher -y \
    ros-noetic-socketcan-interface -y \
    ros-noetic-costmap-queue -y \
    ros-noetic-dwb-critics -y \
    ros-noetic-move-base-msgs -y \
    ros-noetic-rospy-message-converter -y \
    ros-${ROS_DISTRO}-realsense2-camera -y \
    ros-noetic-realsense2-description -y \
    ros-noetic-ros-numpy -y \ 
    python3-tk -y
RUN pip3 install open3d_ros_helper open3d python_tsp



RUN rm -rf robotiq/robotiq_3f*

RUN mkdir -p /urs/src/rubyrhod_ws/src/
RUN mkdir -p /urs/src/rubyrhod_ws/build/
RUN mkdir -p /urs/src/rubyrhod_ws/devel/
WORKDIR /urs/src/rubyrhod_ws/
COPY ./ ./src/
RUN source /urs/src/rubyrhod_ws/src/REQUIREMENT.bash
RUN source /opt/ros/noetic/setup.sh
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /urs/src/rubyrhod_ws/; catkin_make'
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /urs/src/rubyrhod_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export ROS_IP=192.168.0.104" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://192.168.0.104:11311" >> ~/.bashrc
#CMD roslaunch behaviors simulation.launch

# launch ros package
#CMD ["launch"]