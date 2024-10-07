# syntax=docker/dockerfile:1

# Dockerfile for development
# Below RUN statements are broken up to take advantage of Docker layer cache.

# (OPTION) Use base Ubuntu 22.04 if a Nvidia GPU is unavailable.
# FROM ubuntu:22.04
FROM nvidia/cuda:11.7.1-cudnn8-runtime-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG="C.UTF-8" LC_ALL="C.UTF-8"
RUN echo 'Etc/UTC' > /etc/timezone \
  && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN apt-get update \
  # Needed to curl and authorize ROS repository key.
  && apt-get install -y curl gnupg lsb-release software-properties-common \
  && apt-get install -y git \
  # Enable universe repositories.
  && add-apt-repository universe

# Add VNC server & noVNC web app for debugging and control.
COPY ./.devcontainer/scripts/desktop-lite-debian.sh /tmp/scripts/desktop-lite-debian.sh
ENV DBUS_SESSION_BUS_ADDRESS="autolaunch:" \
  VNC_RESOLUTION="1440x768x16" \
  VNC_DPI="96" \
  VNC_PORT="5901" \
  NOVNC_PORT="6080" \
  DISPLAY=":1"
RUN bash /tmp/scripts/desktop-lite-debian.sh root password

# Enable openCL support (OpenCV uses it for hardware acceleration).
RUN mkdir -p /etc/OpenCL/vendors && \
  echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

# Curl key to authorize ROS repository.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# ROS Humble's support window is till 2027 (correct as of 23 Nov 2022).
ARG ROS_DISTRO=humble 
ENV ROS_DISTRO=$ROS_DISTRO

RUN apt-get update
RUN apt-get install -y \
  ros-${ROS_DISTRO}-ros-base \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-pip

# Setup colcon mixin and metadata
RUN colcon mixin add default \
  https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
  colcon mixin update && \
  colcon metadata add default \
  https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
  colcon metadata update

# Install Ros Desktop Full
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-desktop-full

# Set default version of Python to be the one ROS Humble uses.
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.10 10

# Install other ROS Packages
RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-urdf-launch \
  ros-${ROS_DISTRO}-moveit-setup-assistant \
  ros-${ROS_DISTRO}-moveit-simple-controller-manager \
  ros-${ROS_DISTRO}-gripper-controllers \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros-testing \
  ros-${ROS_DISTRO}-graph-msgs \
  ros-${ROS_DISTRO}-rviz-visual-tools \
  ros-${ROS_DISTRO}-moveit-planners \
  ros-${ROS_DISTRO}-joint-state-broadcaster \
  ros-${ROS_DISTRO}-joint-trajectory-controller \
  ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
  ros-${ROS_DISTRO}-moveit-resources-panda-description \
  ros-${ROS_DISTRO}-camera-calibration \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-nav2-rviz-plugins

# Install and generate moveit files
RUN apt install -y python3-colcon-common-extensions \ 
  python3-colcon-mixin \
  python3-vcstool \
  && colcon mixin update default

# Initialize rosdep package manager.
RUN rosdep init && rosdep update

# RQT comes with useful debugging and control tools.
# RQT's plugin support allows for custom visualizations, tools or control panels.
RUN apt-get install -y ~nros-${ROS_DISTRO}-rqt*

# Curl key to authorize Docker repository.
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add - 2>/dev/null \
  && add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable"

# Install Docker CLI tools (not including daemon).
RUN apt-get update \
  && apt-get install -y docker-ce-cli \
  && pip install docker-compose

# Create a ros user with sudo access
RUN addgroup ros
RUN useradd -m -s /bin/bash -g ros ros
RUN echo "ros:ros" | /usr/sbin/chpasswd
RUN echo "ros    ALL=(ALL) ALL" >> /etc/sudoers

# Needed for Dev Container lifecycle hooks to run.
COPY ./.devcontainer /tmp/.devcontainer

ENTRYPOINT [ \
  # VNC entrypoint
  "/usr/local/share/desktop-init.sh" \
  ]

# Make /bin/sh launch bash instead.
ENV ENV=\$HOME/.shrc
RUN echo "exec bash" >> ~/.shrc

# Ensure RQT icons show up.
RUN mkdir ~/.icons && ln -s /usr/share/icons/Tango ~/.icons/hicolor

# Set non-root user as default user
USER ${USERNAME}