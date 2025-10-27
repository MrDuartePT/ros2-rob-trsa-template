#!/bin/bash
# `postStart.sh` is called whenever the Dev Container starts.
# It can be used for misc tasks (e.g., ensuring dependencies are installed).

source /opt/ros/$ROS_DISTRO/setup.bash

# Due to https://github.com/microsoft/vscode-remote-release/issues/6683,
# we have to disable git's repository trust feature.
# Due to https://github.com/microsoft/vscode-remote-release/issues/6810#issuecomment-1159354677,
# this cannot be done in the Dockerfile (else VS Code doesn't configure `.gitconfig`).
git config --global safe.directory "*"

# Ensure submodules are cloned; Doesn't affect already cloned ones.
git submodule update --init --recursive

# Ensure dependencies are installed.
rosdep install --ignore-src --from-path "/home/$USERNAME/rob_ws" -y
rosdep install --ignore-src --from-path "/home/$USERNAME/trsa_ws" -y
rosdep install --ignore-src --from-path "/home/$USERNAME/isaac_ros-dev" -y || continue
pip install -r "$WORKSPACE_ROOT/requirements.txt"

# Install IsaacLab and IsaacSim if isaac_ros-dev exist
if [ -d "/home/$USERNAME/isaac_ros-dev" ]; then
    ln -sf /home/$USERNAME/isaac_ros-dev "$WORKSPACE_ROOT/"
    echo "export ISAAC_ROS_WS=/home/$USERNAME/isaac_ros-dev" >> ~/.bashrc
    echo "Installing Isaac Sim and Isaac Lab"
    $WORKSPACE_ROOT/.devcontainer/scripts/issaclab-pyenv.sh
    echo 'alias issacload="source ~/isaac_ros-dev/env_isaacsim/bin/activate"' >> ~/.bashrc
fi