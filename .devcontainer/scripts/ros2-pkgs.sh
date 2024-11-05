#!/usr/bin/env bash
#-------------------------------------------------------------------------------------------------------------
# This scripts manually install Ros Packages for AArch64
# Since Ubuntu 22.04 not package some of them on arm

if [ "$BUILDARCH" = "arm64" ]; then
    ## Install gazebo from ppa
    sudo add-apt-repository ppa:openrobotics/gazebo11-non-amd64
    sudo apt update

    sudo apt install -y gazebo

    ## Internal Workspace for missing packages
    mkdir -p /opt/ros-internal/src
    cd /opt/ros-internal

    source /opt/ros/$ROS_DISTRO/setup.sh

    ## ros-humble-gazebo_ros_pkgs
    wget https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/ros2/gazebo_ros_pkgs.repos
    sed -i '/vision_opencv/!b;n;s/version: ros2/version: humble/' gazebo_ros_pkgs.repos
    vcs import src < gazebo_ros_pkgs.repos

    vcs custom --args checkout humble
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
else
    apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
fi