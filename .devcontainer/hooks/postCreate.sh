#!/bin/sh
# `postCreate.sh` is called when the Dev Container is first created.
# It can be used for setup steps outside the Dockerfile.

. /opt/ros/$ROS_DISTRO/setup.sh
. /usr/share/gazebo/setup.sh

# Source ros-internal workspace if exist (For AArch64 machines)
if [ -d "/opt/ros-internal" ]; then
    source /opt/ros-internal/setup.bash
fi

# Auto-activate ROS whenever bash shell is opened.
echo "source /opt/ros/$ROS_DISTRO/setup.bash\nsource /usr/share/gazebo/setup.bash" >> ~/.bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash\nsource /usr/share/gazebo/setup.bash" >> /home/${USERNAME}/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc

# Symlink `./rob_ws` and `./trsa_ws` mount point to workspace folder for convenience.
ln -sf /home/vscode/rob_ws "$WORKSPACE_ROOT/"
ln -sf /home/vscode/trsa_ws "$WORKSPACE_ROOT/"

# Something deleted the package indexes so we re-download them for convenience.
sudo apt-get update
sudo rosdep update
