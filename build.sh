#!/bin/bash
# shellcheck disable=SC1091

# Store the current working directory
original_cwd=$(pwd)

# Directories for the core and onboard workspaces
CORE_WS="/root/dev/robosub-ros2/core"
ONBOARD_WS="/root/dev/robosub-ros2/onboard"

# Function to remove paths from an environment variable
remove_paths_from_env_var() {
    env_var_name=$1
    workspace_dir=$2

    # Get the value of the environment variable
    env_var_value=$(eval echo \$"$env_var_name")

    if [ -n "$env_var_value" ]; then
        new_env_var_value=$(echo "$env_var_value" | tr ':' '\n' | grep -v "$workspace_dir" | tr '\n' ':')
        # Remove trailing colon
        new_env_var_value=${new_env_var_value%:}
        export "$env_var_name"="$new_env_var_value"
    fi
}

# Function to clean workspace (build, install, log)
clean_workspace() {
    workspace_dir=$1
    echo "Cleaning workspace: $workspace_dir"
    rm -rf "$workspace_dir/build" "$workspace_dir/install" "$workspace_dir/log"

    # Remove paths from env variables used by colcon that include the workspace_dir
    remove_paths_from_env_var "AMENT_PREFIX_PATH" "$workspace_dir"
    remove_paths_from_env_var "CMAKE_PREFIX_PATH" "$workspace_dir"
    remove_paths_from_env_var "COLCON_PREFIX_PATH" "$workspace_dir"
}

# Function to build workspace or package
build_workspace() {
    workspace_dir=$1
    package_name=$2
    debug_mode=$3

    cd "$workspace_dir" || exit

    build_cmd="colcon build"

    # Don't use symlink install for core workspace
    if [ "$workspace_dir" != "$CORE_WS" ]; then
        build_cmd="$build_cmd --symlink-install"
    fi

    # Add package-specific build if specified
    if [ -n "$package_name" ]; then
        build_cmd="$build_cmd --packages-select $package_name"
        echo "Building package '$package_name' in workspace: $workspace_dir"
    else
        echo "Building workspace: $workspace_dir"
    fi

    # Add debug flags if requested
    if [ "$debug_mode" == true ]; then
        build_cmd="$build_cmd --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
        echo "Debug mode enabled: Adding RelWithDebInfo build type to CMake."
    fi

    $build_cmd --executor sequential

    source install/setup.bash
}

# Main script logic
debug_mode=false

# Check if --debug is specified and validate against 'clean'
for arg in "$@"; do
    if [ "$arg" == "--debug" ]; then
        for check_arg in "$@"; do
            if [ "$check_arg" == "clean" ]; then
                echo "Error: --debug flag cannot be used with the 'clean' command."
                return 1
            fi
        done
        debug_mode=true
        # Remove --debug from arguments to avoid interference
        set -- "${@/--debug/}"
        break
    fi
done

if [ "$1" == "core" ]; then
    # Build all packages in core workspace
    build_workspace "$CORE_WS" "" "$debug_mode"

elif [ "$1" == "onboard" ]; then
    # Build all packages in onboard workspace
    build_workspace "$ONBOARD_WS" "" "$debug_mode"

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
    build_workspace "$ONBOARD_WS" "$1" "$debug_mode"
else
    # Build all packages in both core and onboard workspaces
    build_workspace "$CORE_WS" "" "$debug_mode"
    build_workspace "$ONBOARD_WS" "" "$debug_mode"
fi

# Reload bashrc and return to original directory
source /root/.bashrc
cd "$original_cwd" || exit