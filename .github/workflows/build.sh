#!/bin/bash
# shellcheck disable=SC1091

set -e

source /root/ros_bashrc.sh

source /root/dev/robosub-ros2/build.sh clean
source /root/dev/robosub-ros2/build.sh
exit 0

# Directories for the core and onboard workspaces
CORE_WS="/root/dev/robosub-ros2/core"
ONBOARD_WS="/root/dev/robosub-ros2/onboard"

# Delete the build, install, and log directories in the workspace
clean_workspace() {
    workspace_dir=$1
    echo "Cleaning workspace: $workspace_dir"
    rm -rf "$workspace_dir/build" "$workspace_dir/install" "$workspace_dir/log"
}

# Build the workspace
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