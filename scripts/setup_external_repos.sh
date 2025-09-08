#!/bin/bash


# OUSTER

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


# RealSense

# https://github.com/IntelRealSense/realsense-ros
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
sudo apt install apt-transport-https ros-jazzy-librealsense2*
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg python3-rosdep
git clone -b ros2-master https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y



colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release