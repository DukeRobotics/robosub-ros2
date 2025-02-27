#!/bin/bash
# shellcheck disable=SC1090,SC1091,SC2164

# `colcon build` triggers this warning when building Python packages, which is not an issue
export PYTHONWARNINGS="ignore:easy_install command is deprecated"

# Use colorized output in ROS commands
# For example, warning messages will be in yellow, error messages in red
export RCUTILS_COLORIZED_OUTPUT=1

# Direct all ROS logs to stdout instead of stderr
# This means any node with output="log" will not print to the terminal
export RCUTILS_LOGGING_USE_STDOUT=1

source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh

# Check if the core setup file exists and source it
if [ -f /home/ubuntu/robosub-ros2/core/install/setup.bash ]; then
    source /home/ubuntu/robosub-ros2/core/install/setup.bash
else
    echo "Core setup.bash not found, skipping."
fi

# Check if the onboard setup file exists and source it
if [ -f /home/ubuntu/robosub-ros2/onboard/install/setup.bash ]; then
    source /home/ubuntu/robosub-ros2/onboard/install/setup.bash
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
alias fox="python3 /home/ubuntu/robosub-ros2/foxglove/foxglove.py"

# Alias to run lint.py
alias lint="python3 /home/ubuntu/robosub-ros2/lint.py"

# Aliases to enable and disable controls
alias enable-controls="ros2 service call /controls/enable std_srvs/srv/SetBool '{data: true}'"
alias disable-controls="ros2 service call /controls/enable std_srvs/srv/SetBool '{data: false}'"

# Source nvm and use version jod
source /home/ubuntu/.nvm/nvm.sh
nvm use lts/jod > /dev/null  # Suppress stdout, only show stderr

# Source python virtual environment
source /home/ubuntu/robosub-ros2/venv.sh activate
