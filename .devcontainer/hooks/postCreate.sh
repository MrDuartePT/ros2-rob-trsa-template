#!/bin/sh
# `postCreate.sh` is called when the Dev Container is first created.
# It can be used for setup steps outside the Dockerfile.

. /opt/ros/$ROS_DISTRO/setup.sh
. /usr/share/gazebo/setup.bash

# Auto-activate ROS whenever bash shell is opened.
echo "source /opt/ros/$ROS_DISTRO/setup.bash\nsource $WORKSPACE_ROOT/install/local_setup.bash" >> ~/.bashrc

# Something deleted the package indexes so we re-download them for convenience.
apt-get update
rosdep update
