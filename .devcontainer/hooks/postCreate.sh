#!/bin/bash
# `postCreate.sh` is called when the Dev Container is first created.
# It can be used for setup steps outside the Dockerfile.

source /opt/ros/$ROS_DISTRO/setup.bash

# Auto-activate ROS whenever bash shell is opened.
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo 'PATH=$PATH:/usr/local/bin' >> /root/.bashrc
echo 'PATH=$PATH:/usr/local/bin' >> ~/.bashrc

# Source ros-internal workspace if exist (For AArch64 machines)
if [ -d "/opt/ros-internal" ]; then
    source /opt/ros-internal/install/setup.bash
    echo "source /opt/ros-internal/install/setup.bash" >> ~/.bashrc
fi

# Symlink `./rob_ws` and `./trsa_ws` mount point to workspace folder for convenience.
ln -sf /home/${USERNAME}/rob_ws "$WORKSPACE_ROOT/"
ln -sf /home/${USERNAME}/trsa_ws "$WORKSPACE_ROOT/"

if [ -h "$WORKSPACE_ROOT/rob_ws" ] && [ -h "$WORKSPACE_ROOT/trsa_ws" ]; then
    source $WORKSPACE_ROOT/rob_ws/install/setup.bash
    source $WORKSPACE_ROOT/trsa_ws/install/setup.bash
    echo "source $WORKSPACE_ROOT/rob_ws/install/setup.bash" >> ~/.bashrc
    echo "source $WORKSPACE_ROOT/trsa_ws/install/setup.bash" >> ~/.bashrc
fi

# Install IsaacLab and IsaacSim if isaac_ros-dev exist
if [ -h "/home/$USERNAME/isaac_ros-dev" ]; then
    ln -sf /home/$USERNAME/isaac_ros-dev "$WORKSPACE_ROOT/"
    echo "export ISAAC_ROS_WS=/home/$USERNAME/isaac_ros-dev" >> ~/.bashrc
    echo "Installing Isaac Sim and Isaac Lab"
    $WORKSPACE_ROOT/.devcontainer/scripts/issaclab-pyenv.sh
    echo 'alias issacload="source ~/isaac_ros-dev/env_isaacsim/bin/activate"' >> ~/.bashrc
fi

# Add useful allias
echo 'alias ros2_cmake_pkg="ros2 pkg create --build-type ament_cmake"' >> ~/.bashrc
echo 'alias ros2_python_pkg="ros2 pkg create --build-type ament_python"' >> ~/.bashrc

# Something deleted the package indexes so we re-download them for convenience.
sudo apt-get update
rosdep update
