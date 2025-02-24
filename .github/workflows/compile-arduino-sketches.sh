#!/bin/bash
# shellcheck disable=SC1091
# This script is used by GitHub Actions to compile all Arduino sketches
# This script must be run after the onboard workspace has been built

# Exit immediately if a command exits with a non-zero status
# This ensures the exit code of the script is 1 if any command fails, and 0 if all commands succeed
set -e

source /home/ubuntu/ros_bashrc.sh

# Loop through every valid robot name and compile the Arduino sketches for each robot
while read -r line; do
    export ROBOT_NAME="$line"
    echo "::group::Compiling Arduino sketches for robot: $ROBOT_NAME"
    ros2 run offboard_comms arduino compile all
    echo "::endgroup::"
done < "/home/ubuntu/robosub-ros2/robot_names"