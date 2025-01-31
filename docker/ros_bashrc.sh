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
alias thruster-test="ros2 topic pub /controls/thruster_allocs custom_msgs/msg/ThrusterAllocs 'allocs: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]'"

source /root/dev/robosub-ros2/venv.sh activate