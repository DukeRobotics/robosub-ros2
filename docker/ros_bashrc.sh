#!/bin/bash
# shellcheck disable=SC1090,SC1091,SC2164

export PYTHONWARNINGS="ignore:easy_install command is deprecated"

source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh

# Check if the core setup file exists and source it
if [ -f /root/dev/robosub-ros2/core/install/setup.bash ]; then
    source /root/dev/robosub-ros2/core/install/setup.bash
else
    echo "Core setup.bash not found, skipping."
fi

# Check if the onboard setup file exists and source it
if [ -f /root/dev/robosub-ros2/onboard/install/setup.bash ]; then
    source /root/dev/robosub-ros2/onboard/install/setup.bash
else
    echo "Onboard setup.bash not found, skipping."
fi

export _colcon_cd_root=/opt/ros/jazzy

# Alias to start foxglove bridge
alias fg-ws="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=28765"

# Alias to run test_thrusters node in offboard comms
alias test-thrusters="ros2 run offboard_comms test_thrusters"

# Alias to run arduino CLI in offboard comms
alias arduino="ros2 run offboard_comms arduino"

# Alias for foxglove.py CLI
alias fox="python3 /root/dev/robosub-ros2/foxglove/foxglove.py"

# Source nvm and use version jod
source /root/.nvm/nvm.sh
nvm use lts/jod

# Source python virtual environment
source /root/dev/robosub-ros2/venv.sh activate
