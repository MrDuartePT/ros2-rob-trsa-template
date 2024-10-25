# syntax=docker/dockerfile:1

# Dockerfile for development
# Below RUN statements are broken up to take advantage of Docker layer cache.

# (OPTION) Use base Ubuntu 22.04 if a Nvidia GPU is unavailable.
# FROM ubuntu:22.04
FROM nvidia/cuda:11.7.1-cudnn8-runtime-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG="en_US.UTF-8" LC_ALL="en_US.UTF-8" LANGUAGE="en_US.UTF-8"
RUN echo 'Etc/UTC' > /etc/timezone \
  && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN apt-get update \
  # Needed to curl and authorize ROS repository key.
  && apt-get install -y curl sudo gnupg lsb-release software-properties-common \
  && apt-get install -y git \
  # Enable universe repositories.
  && add-apt-repository universe

# Create a vscode user with sudo access
ARG USERNAME=vscode
ENV USERNAME=$USERNAME
RUN addgroup ${USERNAME}
RUN useradd -m -s /bin/bash -g ${USERNAME} ${USERNAME}
RUN echo "${USERNAME}:${USERNAME}" | /usr/sbin/chpasswd
RUN echo "${USERNAME}    ALL=(ALL) ALL" >> /etc/sudoers

# Add VNC server & noVNC web app for debugging and control.
COPY ./.devcontainer/scripts/desktop-lite-debian.sh /tmp/scripts/desktop-lite-debian.sh
ENV DBUS_SESSION_BUS_ADDRESS="autolaunch:" \
  VNC_RESOLUTION="1440x768x16" \
  VNC_DPI="96" \
  VNC_PORT="5901" \
  NOVNC_PORT="6080" \
  DISPLAY=":1"
RUN bash /tmp/scripts/desktop-lite-debian.sh vscode vscode

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
  python3-pip

# Set default version of Python to be the one ROS Humble uses.
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.10 10

# Install other ROS Packages
RUN apt-get update && apt-get install -y \
  ros-humble-joint-state-publisher-gui \
  ros-humble-urdf-launch \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-gripper-controllers \
  ros-humble-ros2-control \
  ros-humble-moveit-planners \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-moveit-resources-panda-description \
  ros-humble-camera-calibration \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization

# Install moveit files
RUN apt install -y python3-colcon-common-extensions \ 
  python3-colcon-mixin \
  python3-vcstool

# Initialize rosdep package manager.
RUN rosdep init && rosdep update

# RQT comes with useful debugging and control tools.
# RQT's plugin support allows for custom visualizations, tools or control panels.
RUN apt-get install -y ros-${ROS_DISTRO}-rqt*

# Curl key to authorize Docker repository.
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add - 2>/dev/null \
  && add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable"

# Install Docker CLI tools (not including daemon).
RUN apt-get update \
  && apt-get install -y docker-ce-cli \
  && pip install docker-compose

# Needed for Dev Container lifecycle hooks to run.
COPY ./.devcontainer /tmp/.devcontainer

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ \
  # VNC entrypoint
  "/usr/local/share/desktop-init.sh" \
  ]

# Make /bin/sh launch bash instead.
ENV ENV=\$HOME/.shrc
RUN echo "exec bash" >> ~/.shrc

# Ensure RQT icons show up.
RUN mkdir ~/.icons && ln -s /usr/share/icons/Tango ~/.icons/hicolor
