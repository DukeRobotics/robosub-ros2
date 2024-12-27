#!/bin/bash
# shellcheck disable=SC1091
# This script is used by GitHub Actions to install all Foxglove extensions

# Exit immediately if a command exits with a non-zero status
# This ensures the exit code of the script is 1 if any command fails, and 0 if all commands succeed
set -e

source /root/ros_bashrc.sh

/root/dev/venv/bin/python3 /root/dev/robosub-ros2/foxglove/foxglove.py install --skip-build
