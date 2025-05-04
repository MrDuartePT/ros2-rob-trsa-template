# syntax=docker/dockerfile:1

# Dockerfile for development
# Below RUN statements are broken up to take advantage of Docker layer cache.

FROM ubuntu:22.04
ARG MACOS_BUILD
ENV TARGETARCH=${TARGETARCH}

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG="en_US.UTF-8" LC_ALL="en_US.UTF-8" LANGUAGE="en_US.UTF-8"
RUN echo 'Etc/UTC' > /etc/timezone \
  && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN apt-get update && apt-get -y upgrade \
  # Needed to curl and authorize ROS repository key.
  && apt-get install -y curl wget sudo gnupg lsb-release software-properties-common \
  && apt-get install -y git \
  # Enable universe repositories.
  && add-apt-repository universe

# Create a vscode user with sudo access
ARG USERNAME=vscode
ENV USERNAME=$USERNAME
RUN addgroup ${USERNAME}
RUN useradd -m -s /bin/bash -g ${USERNAME} ${USERNAME}
RUN usermod -a -G sudo ${USERNAME}
RUN echo "${USERNAME}:${USERNAME}" | /usr/sbin/chpasswd
RUN echo "${USERNAME}    ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Create input user
RUN groupadd -g 97 input1
RUN usermod -a -G input1 ${USERNAME}

ENV DBUS_SESSION_BUS_ADDRESS="autolaunch:"
ENV VNC_RESOLUTION="1920x1080x32"
ENV VNC_DPI="96"
ENV VNC_PORT="5901"
ENV NOVNC_PORT="6080"
ENV DISPLAY=":1"

COPY ./.devcontainer/scripts/desktop-lite-debian.sh /tmp/scripts/desktop-lite-debian.sh
COPY ./.devcontainer/scripts/gpu-deps.sh /tmp/scripts/gpu-deps.sh
COPY ./.devcontainer/scripts/entrypoint.sh /tmp/scripts/entrypoint.sh

# Add VNC server & noVNC web app for enviroment in MacOS
RUN if [ "$MACOS_BUILD" = "true" ]; then \
      bash /tmp/scripts/desktop-lite-debian.sh vscode vscode; \
fi

# Install GPU dependencies
RUN if [ "$TARGETARCH" = "amd64" ]; then \
      bash /tmp/scripts/gpu-deps.sh; \
fi

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
  python3-vcstool \
  python3-pip

# Setup colcon mixin and metadata
RUN colcon mixin add default \
  https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
  colcon mixin update && \
  colcon metadata add default \
  https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
  colcon metadata update

# Install Ros Desktop Full
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop-full

# Set default version of Python to be the one ROS Humble uses.
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.10 10

# RQT comes with useful debugging and control tools.
# RQT's plugin support allows for custom visualizations, tools or control panels.
RUN apt-get install -y ros-${ROS_DISTRO}-rqt*

# Groot 2 (no AppImage for Arm64 use Box64)
COPY ./.devcontainer/scripts/groot2.sh /tmp/scripts/groot2.sh
RUN bash /tmp/scripts/groot2.sh

# Install other ROS Packages
RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-py-binding-tools \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-urdf-launch \
  ros-${ROS_DISTRO}-gripper-controllers \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros-testing \
  ros-${ROS_DISTRO}-graph-msgs \
  ros-${ROS_DISTRO}-rviz-visual-tools \
  ros-${ROS_DISTRO}-urdf-tutorial \
  ros-${ROS_DISTRO}-joint-state-broadcaster \
  ros-${ROS_DISTRO}-joint-trajectory-controller \
  ros-${ROS_DISTRO}-camera-calibration \
  ros-${ROS_DISTRO}-behaviortree-cpp

# Moveit packages
RUN apt-get install -y \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-common \
  ros-${ROS_DISTRO}-moveit-resources \
  ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
  ros-${ROS_DISTRO}-moveit-resources-panda-description \
  ros-${ROS_DISTRO}-moveit-ros-perception \
  ros-${ROS_DISTRO}-moveit-runtime \
  ros-${ROS_DISTRO}-moveit-setup-assistant \
  ros-${ROS_DISTRO}-moveit-simple-controller-manager \
  ros-${ROS_DISTRO}-moveit-servo \
  ros-${ROS_DISTRO}-moveit-visual-tools \
  ros-${ROS_DISTRO}-moveit-planners

# Nax2 packages
RUN apt-get install -y \ 
  ros-${ROS_DISTRO}-nav2-controller \
  ros-${ROS_DISTRO}-nav2-smoother \
  ros-${ROS_DISTRO}-nav2-behaviors \
  ros-${ROS_DISTRO}-nav2-dwb-controller \
  ros-${ROS_DISTRO}-nav2-navfn-planner \
  ros-${ROS_DISTRO}-nav2-bt-navigator \
  ros-${ROS_DISTRO}-nav2-lifecycle-manager \
  ros-${ROS_DISTRO}-nav2-rviz-plugins \
  ros-${ROS_DISTRO}-nav2-planner \
  ros-${ROS_DISTRO}-nav2-waypoint-follower \
  ros-${ROS_DISTRO}-nav2-velocity-smoother \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-twist-mux

# Some ros package are not available on AArch64 on ubuntu22.04
# To solve that problem will clone them to internal workspace
# On amd64 will still use the ubuntu packages
COPY ./.devcontainer/scripts/ros2-pkgs.sh /tmp/scripts/ros2-pkgs.sh
COPY ./.devcontainer/scripts/turtlebot3-gazebo.repos /tmp/scripts/turtlebot3-gazebo.repos
RUN bash /tmp/scripts/ros2-pkgs.sh

# Nvidia Isaac ROS packages
RUN curl -sSL https://isaac.download.nvidia.com/isaac-ros/repos.key -o /usr/share/keyrings/isaac-ros.key
RUN echo "deb [signed-by=/usr/share/keyrings/isaac-ros.key] https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" > /etc/apt/sources.list.d/isaac-ros.list

RUN curl -sSL https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/librealsense-intel.list

RUN apt update

RUN apt-get install -y \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-isaac-ros-common \
    ros-${ROS_DISTRO}-isaac-ros-argus-camera \
    librealsense2-utils \
    librealsense2-dev
# Isaac SIM not included in docker image mount folder from the host

# Initialize rosdep package manager.
RUN rosdep init && rosdep update

# Curl key to authorize Docker repository.
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add - 2>/dev/null \
  && add-apt-repository "deb [arch=$(dpkg --print-architecture)] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) stable"

# Install Docker CLI tools (not including daemon).
RUN apt-get update \
  && apt-get install -y docker-ce-cli \
  && pip install docker-compose

RUN chmod +x /tmp/scripts/entrypoint.sh
ENTRYPOINT ["/tmp/scripts/entrypoint.sh"]

# Make /bin/sh launch bash instead.
ENV ENV=\$HOME/.shrc
RUN echo "exec bash" >> ~/.shrc

# Ensure RQT icons show up.
RUN mkdir ~/.icons && ln -s /usr/share/icons/Tango ~/.icons/hicolor

# Update all the packages
RUN apt-get update && apt-get upgrade -y

# Set non-root user as default user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Setup colcon mixin and metadata in vscode user
RUN colcon mixin add default \
  https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
  colcon mixin update && \
  colcon metadata add default \
  https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
  colcon metadata update