# syntax=docker/dockerfile:1

# Dockerfile for development
# Below RUN statements are broken up to take advantage of Docker layer cache.

FROM ubuntu:24.04
ARG MACOS_BUILD
ARG TARGETARCH

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG="en_US.UTF-8" LC_ALL="en_US.UTF-8" LANGUAGE="en_US.UTF-8"
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN apt-get update && apt-get -y upgrade \
  # Needed to curl and authorize ROS repository key. (226MB)
  && apt-get install -y git curl wget sudo gnupg lsb-release software-properties-common \
  # Enable universe repositories.
  && add-apt-repository -y universe

# Rename default ubuntu user to vscode and give sudo access
ARG USERNAME=vscode
ENV USERNAME=$USERNAME
RUN usermod -l ${USERNAME} -d /home/${USERNAME} -m ubuntu && \
    groupmod -n ${USERNAME} ubuntu && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME}:${USERNAME}" | chpasswd && \
    echo "${USERNAME}    ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Create input user
RUN groupadd -g 97 input1
RUN usermod -a -G input1 ${USERNAME}

ENV DBUS_SESSION_BUS_ADDRESS="autolaunch:"
ENV VNC_RESOLUTION="1920x1080x32"
ENV VNC_DPI="96"
ENV VNC_PORT="5901"
ENV NOVNC_PORT="6080"
ENV DISPLAY=":1"

# Add VNC server & noVNC web app for enviroment in MacOS
COPY ./.devcontainer/scripts/desktop-lite-debian.sh /tmp/scripts/desktop-lite-debian.sh
RUN if [ "$MACOS_BUILD" = "true" ]; then \
    bash /tmp/scripts/desktop-lite-debian.sh vscode vscode; \
    rm /tmp/scripts/desktop-lite-debian.sh; \
fi

# Install GPU dependencies
COPY ./.devcontainer/scripts/gpu-deps.sh /tmp/scripts/gpu-deps.sh
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    bash /tmp/scripts/gpu-deps.sh; \
    rm /tmp/scripts/gpu-deps.sh; \
fi

# Install Ros apt repository package
RUN set -e && \
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# ROS Jazzy's support window is till 2029 (correct as of 4 Feb 2025).
ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=$ROS_DISTRO

# Ros-Base depedencys (625MB)
RUN apt-get update && apt-get install -y \
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

# Install Ros Desktop Full and RQT (3119 MB)
# Set default version of Python to be the one ROS Humble uses.
# RQT comes with useful debugging and control tools.
# RQT's plugin support allows for custom visualizations, tools or control panels.
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop-full ros-${ROS_DISTRO}-rqt* && \
    mkdir -p /home/${USERNAME}/.config/pip && \
    printf "[global]\nbreak-system-packages = true\n" > /home/${USERNAME}/.config/pip/pip.conf && \
    chown vscode:vscode /home/${USERNAME}/.config/pip/pip.conf

# Groot 2 (no AppImage for Arm64 use Box64)
COPY ./.devcontainer/scripts/groot2.sh /tmp/scripts/groot2.sh
RUN bash /tmp/scripts/groot2.sh

# Install other ROS Packages (12.1MB)
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

# Moveit packages (114MB)
RUN apt-get update && apt-get install -y \
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
  ros-${ROS_DISTRO}-moveit-planners \
  ros-${ROS_DISTRO}-moveit-py

# NaV2 packages (402MB)
RUN apt-get update && apt-get install -y \
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

# Update to Turtlebot4 (79.2MB)
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-irobot-create-nodes \
    ros-${ROS_DISTRO}-turtlebot4-description \
    ros-${ROS_DISTRO}-turtlebot4-msgs \
    ros-${ROS_DISTRO}-turtlebot4-navigation \
    ros-${ROS_DISTRO}-turtlebot4-node \
    ros-${ROS_DISTRO}-turtlebot4-simulator \
    ros-${ROS_DISTRO}-turtlebot4-desktop

# Nvidia Isaac ROS and Intel Realsense packages (291MB)
# Isaac SIM not included in docker image (can be install using pip requriment file
RUN if [ "$MACOS_BUILD" = "false" ]; then \
    curl -sSL https://isaac.download.nvidia.com/isaac-ros/repos.key -o /usr/share/keyrings/isaac-ros.key && \
    echo "deb [signed-by=/usr/share/keyrings/isaac-ros.key] https://isaac.download.nvidia.com/isaac-ros/release-4 $(lsb_release -cs) main" > /etc/apt/sources.list.d/isaac-ros.list && \
    curl -sSL https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/librealsense-intel.list && \
    apt update && apt-get install -y libnvvpi4 vpi4-dev vpi4-samples && \ 
    apt-get install -y ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-isaac-ros-common isaac-ros-cli; \
fi

RUN if [ && "$TARGETARCH" = "amd64" ]; then \
    apt update && apt-get install -y librealsense2-utils librealsense2-dev; \
fi

# Initialize rosdep package manager.
RUN rosdep init && rosdep update

# Install Docker CLI tools (not including daemon).
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add - 2>/dev/null \
  && add-apt-repository "deb [arch=$(dpkg --print-architecture)] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) stable" && \
  apt-get update && apt-get install -y docker-ce-cli docker-compose-plugin

RUN if [ "$MACOS_BUILD" = "true" ]; then \
      printf '%s\n' '#!/bin/bash' 'set -e' 'exec /usr/local/share/desktop-init.sh "$@"' > /entrypoint.sh; \
    else \
      printf '%s\n' '#!/bin/bash' 'set -e' 'exec "$@"' > /entrypoint.sh; \
    fi && \
    chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

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