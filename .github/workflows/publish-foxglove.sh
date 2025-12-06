#!/bin/bash
# shellcheck disable=SC1091
# This script is used by GitHub Actions to publish all custom Foxglove extensions

set -e

source /home/ubuntu/ros_bashrc.sh

/home/ubuntu/venv/bin/python3 /home/ubuntu/robosub-ros2/foxglove/foxglove.py publish --github-action
