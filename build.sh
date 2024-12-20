#!/bin/bash
# shellcheck disable=SC1091

# Store the current working directory
original_cwd=$(pwd)

# Directories for the core and onboard workspaces
CORE_WS="/root/dev/robosub-ros2/core"
ONBOARD_WS="/root/dev/robosub-ros2/onboard"

# Function to clean workspace (build, install, log)
clean_workspace() {
    workspace_dir=$1
    echo "Cleaning workspace: $workspace_dir"
    rm -rf "$workspace_dir/build" "$workspace_dir/install" "$workspace_dir/log"
}

# Function to build workspace or package
build_workspace() {
    workspace_dir=$1
    package_name=$2

    cd "$workspace_dir" || exit

    build_cmd="colcon build"

    # Don't use symlink install for core workspace
    if [ "$workspace_dir" != "$CORE_WS" ]; then
        build_cmd="$build_cmd --symlink-install"
    fi

    if [ -n "$package_name" ]; then
        build_cmd="$build_cmd --packages-select $package_name"
        echo "Building package '$package_name' in workspace: $workspace_dir"
    else
        echo "Building workspace: $workspace_dir"
    fi

    $build_cmd --executor sequential

    source install/setup.bash
}

# Main script logic
if [ "$1" == "core" ]; then
    # Build all packages in core workspace
    build_workspace "$CORE_WS"

elif [ "$1" == "onboard" ]; then
    # Build all packages in onboard workspace
    build_workspace "$ONBOARD_WS"

elif [ "$1" == "clean" ]; then
    # Clean workspaces
    if [ "$2" == "core" ]; then
        clean_workspace "$CORE_WS"
    elif [ "$2" == "onboard" ]; then
        clean_workspace "$ONBOARD_WS"
    else
        clean_workspace "$CORE_WS"
        clean_workspace "$ONBOARD_WS"
    fi

elif [ -n "$1" ]; then
    # Build a specific package in the onboard workspace
    build_workspace "$ONBOARD_WS" "$1"
else
    # Build all packages in both core and onboard workspaces
    build_workspace "$CORE_WS"
    build_workspace "$ONBOARD_WS"
fi

# Reload bashrc and return to original directory
source /root/.bashrc
cd "$original_cwd" || exit