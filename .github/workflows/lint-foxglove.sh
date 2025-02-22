#!/bin/bash
# shellcheck disable=SC1091
# This script is used by GitHub Actions to lint the Foxglove monorepo

# Exit immediately if a command exits with a non-zero status
# This ensures the exit code of the script is 1 if any command fails, and 0 if all commands succeed
set -e

source /home/ubuntu/ros_bashrc.sh

/home/ubuntu/venv/bin/python3 /home/ubuntu/robosub-ros2/foxglove/foxglove.py lint
