#!/bin/bash
# shellcheck disable=SC1090,SC1091

export PYTHONWARNINGS="ignore:easy_install command is deprecated"

# To be appended to /root/.bashrc
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

source /root/dev/venv/bin/activate
export PYTHONPATH=$PYTHONPATH:/root/dev/venv/lib/python3.12/site-packages