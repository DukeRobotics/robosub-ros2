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

# Main script logic
if [ "$1" == "core" ]; then
    # Build all packages in core workspace
    cd "$CORE_WS"
    colcon build --symlink-install --executor sequential
    source install/setup.bash

elif [ "$1" == "onboard" ]; then
    # Build all packages in onboard workspace
    cd "$ONBOARD_WS"
    colcon build --symlink-install --executor sequential
    source install/setup.bash

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
    cd "$ONBOARD_WS"
    colcon build --symlink-install --packages-select "$1"
    source install/setup.bash

else
    # Build all packages in both core and onboard workspaces
    cd "$CORE_WS"
    colcon build --executor sequential
    source install/setup.bash
    cd "$ONBOARD_WS"
    colcon build --symlink-install --executor sequential
    source install/setup.bash
fi

# Reload bashrc and return to original directory
source ~/.bashrc
cd "$original_cwd"