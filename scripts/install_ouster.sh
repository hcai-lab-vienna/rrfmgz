#!/bin/bash

# https://github.com/ouster-lidar/ouster-ros/tree/ros2

sudo apt install -y             \
    ros-$ROS_DISTRO-pcl-ros     \
    ros-$ROS_DISTRO-tf2-eigen   \
    ros-$ROS_DISTRO-rviz2       \
    build-essential             \
    libeigen3-dev               \
    libjsoncpp-dev              \
    libspdlog-dev               \
    libcurl4-openssl-dev        \
    cmake                       \
    python3-colcon-common-extensions

git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git src/ouster-ros

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ouster_ros
