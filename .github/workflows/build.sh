#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash
export PYTHONWARNINGS="ignore:easy_install command is deprecated"

# Directories for the core and onboard workspaces
CORE_WS="/root/dev/robosub-ros2/core"
ONBOARD_WS="/root/dev/robosub-ros2/onboard"

# Function to clean workspace (build, install, log)
clean_workspace() {
    workspace_dir=$1
    echo "Cleaning workspace: $workspace_dir"
    rm -rf "$workspace_dir/build" "$workspace_dir/install" "$workspace_dir/log"
}

build_workspace() {
    workspace_dir=$1
    echo "Building workspace: $workspace_dir"
    cd "$workspace_dir"
    colcon build --symlink-install
    source install/setup.bash
}

clean_workspace "$CORE_WS"
clean_workspace "$ONBOARD_WS"

build_workspace "$CORE_WS"
build_workspace "$ONBOARD_WS"