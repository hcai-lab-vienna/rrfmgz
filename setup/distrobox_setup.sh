#!/bin/bash

### ON HOST DO ###
# distrobox create --image ubuntu:24.04 --name jazzy --home ~/Distrobox/jazzy #--nvidia
##################

sudo apt update
sudo apt upgrade -y

### ROS2 JAZZY ###
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt install locales software-properties-common curl -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install ros-dev-tools ros-jazzy-desktop -y

### additional packages
sudo apt install $(cat apt_package_list.txt) -y

### repo(s)
git clone git@github.com:3ernhard/rrfmgz.git ~/rrfmgz

### Temporary fix for:
###	https://github.com/gazebosim/ros_gz/issues/774
###	https://github.com/gazebosim/ros_gz/pull/775
# sudo mv ./ros_gz_bridge.py /opt/ros/jazzy/lib/python3.12/site-packages/ros_gz_bridge/actions/
