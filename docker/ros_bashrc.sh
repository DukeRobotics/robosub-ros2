#!/bin/bash
# shellcheck disable=SC1090,SC1091

export PYTHONWARNINGS="ignore:easy_install command is deprecated"

# To be appended to /root/.bashrc
source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
source /root/dev/robosub-ros2/core2/install/setup.bash
source /root/dev/robosub-ros2/onboard2/install/setup.bash

export _colcon_cd_root=/opt/ros/jazzy
