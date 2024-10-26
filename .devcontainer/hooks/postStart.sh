#!/bin/sh
# `postStart.sh` is called whenever the Dev Container starts.
# It can be used for misc tasks (e.g., ensuring dependencies are installed).

. /opt/ros/$ROS_DISTRO/setup.sh
. /usr/share/gazebo/setup.sh

# Due to https://github.com/microsoft/vscode-remote-release/issues/6683,
# we have to disable git's repository trust feature.
# Due to https://github.com/microsoft/vscode-remote-release/issues/6810#issuecomment-1159354677,
# this cannot be done in the Dockerfile (else VS Code doesn't configure `.gitconfig`).
git config --global safe.directory "*"

# Ensure submodules are cloned; Doesn't affect already cloned ones.
git submodule update --init --recursive

# Ensure dependencies are installed.
sudo rosdep install --ignore-src --from-path "/home/$USERNAME/rob_ws" -y
sudo rosdep install --ignore-src --from-path "/home/$USERNAME/trsa_ws" -y
sudo pip install -r "$WORKSPACE_ROOT/requirements.txt"
