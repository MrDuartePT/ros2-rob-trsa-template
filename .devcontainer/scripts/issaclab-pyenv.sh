#!/usr/bin/env bash
#-------------------------------------------------------------------------------------------------------------
# This script installs Isaac Sim and Isaac Lab using a Python virtual environment
#-------------------------------------------------------------------------------------------------------------

set -e
echo ""
echo "====================================================================="
echo "Installing Isaac Sim + Isaac Lab Python Environment"
echo "====================================================================="
echo "Environment Path: isaac_ros-dev/env_isaacsim"
echo "Python Version: 3.11"
echo "---------------------------------------------------------------------"
echo ""

# --- Install Python and create venv ---
if ! command -v python3.11 &> /dev/null; then
    sudo apt-get update -y && sudo apt-get install -y python3.11 python3.11-venv
fi

if [ -d "isaac_ros-dev/env_isaacsim" ]; then
    echo "Existing environment detected at isaac_ros-dev/env_isaacsim."
    echo "Skipping environment creation and package installation."
    echo ""
    echo "To activate later, run:"
    echo "    source isaac_ros-dev/env_isaacsim/bin/activate"
    echo "    or use the `issacload` alias"
    echo ""
    echo "    Or, create that handy alias if dosent exit:"
    echo "    echo 'alias issacload="source ~/isaac_ros-dev/env_isaacsim/bin/activate"' >> ~/.bashrc"
    echo "====================================================================="
    echo ""
fi

python3.11 -m venv isaac_ros-dev/env_isaacsim
source isaac_ros-dev/env_isaacsim/bin/activate

# Pip Install for Isaac Sim and Isaac Lab dependencies (you can edit the version)
pip install --upgrade pip
pip install isaaclab[isaacsim,all]==2.2.0 --extra-index-url https://pypi.nvidia.com
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu129
pip install git+https://github.com/isaac-sim/rl_games.git@python3.11

# --- Fancy End Banner ---
echo ""
echo "====================================================================="
echo " Installation Complete!"
echo "====================================================================="
echo "Isaac Sim + Isaac Lab environment is ready to use."
echo "Location: isaac_ros-dev/env_isaacsim"
echo ""
echo "To activate later, run:"
echo "    source isaac_ros-dev/env_isaacsim/bin/activate"
echo "    or use the `issacload` alias"
echo ""
echo "    Or, create that handy alias if dosent exit:"
echo "    echo 'alias issacload="source ~/isaac_ros-dev/env_isaacsim/bin/activate"' >> ~/.bashrc"
echo "====================================================================="
echo ""