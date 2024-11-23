#!/bin/bash
# shellcheck disable=SC1091

set -e

source /root/ros_bashrc.sh

source /root/dev/robosub-ros2/build.sh clean
source /root/dev/robosub-ros2/build.sh