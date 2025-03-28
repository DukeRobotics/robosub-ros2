#!/bin/bash
# shellcheck disable=SC1091
# This script is used by GitHub Actions to build all ROS 2 packages

# Exit immediately if a command exits with a non-zero status
# This ensures the exit code of the script is 1 if any command fails, and 0 if all commands succeed
set -e

source /home/ubuntu/ros_bashrc.sh

source /home/ubuntu/robosub-ros2/build.sh clean
source /home/ubuntu/robosub-ros2/build.sh