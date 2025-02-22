#!/bin/bash
# shellcheck disable=SC1091

# Store the current working directory
original_cwd=$(pwd)

# Directories for the core and onboard workspaces
CORE_WS="/home/ubuntu/robosub-ros2/core"
ONBOARD_WS="/home/ubuntu/robosub-ros2/onboard"

# Function to check if a package exists in a workspace
package_exists() {
    package_name=$1
    workspace_dir=$2

    # Run colcon list and check if the package exists
    colcon list --base-path "$workspace_dir" | awk '{print $1}' | grep -q "^$package_name$";
}

# Function to remove paths from an environment variable
remove_paths_from_env_var() {
    env_var_name=$1
    workspace_dir=$2
    package_name=$3

    # If package_name is specified, remove the install path for that package
    # Otherwise, remove the entire workspace directory
    if [ -n "$package_name" ]; then
        path_to_remove="$workspace_dir/install/$package_name"
    else
        path_to_remove="$workspace_dir"
    fi

    # Get the value of the environment variable
    env_var_value=$(eval echo \$"$env_var_name")

    if [ -n "$env_var_value" ]; then
        new_env_var_value=$(echo "$env_var_value" | tr ':' '\n' | grep -v "$path_to_remove" | tr '\n' ':')
        new_env_var_value=${new_env_var_value%:}  # Remove trailing colon
        export "$env_var_name"="$new_env_var_value"
    fi
}

# Function to clean workspace (build, install, log)
clean_workspace() {
    workspace_dir=$1
    package_name=$2

    if [ -n "$package_name" ]; then
        echo "Cleaning package '$package_name' in workspace: $workspace_dir"
        rm -rf "$workspace_dir/build/$package_name" "$workspace_dir/install/$package_name"  # Packages don't have a log directory
    else
        echo "Cleaning workspace: $workspace_dir"
        rm -rf "$workspace_dir/build" "$workspace_dir/install" "$workspace_dir/log"
    fi

    # Remove paths from env variables used by colcon that include the workspace_dir
    remove_paths_from_env_var "AMENT_PREFIX_PATH" "$workspace_dir" "$package_name"
    remove_paths_from_env_var "CMAKE_PREFIX_PATH" "$workspace_dir" "$package_name"
    remove_paths_from_env_var "COLCON_PREFIX_PATH" "$workspace_dir" "$package_name"

    # If only cleaning a package, source the workspace setup.bash to update the environment
    if [ -n "$package_name" ]; then
        source "$workspace_dir/install/setup.bash"
    fi
}

# Function to build workspace or package
build_workspace() {
    workspace_dir=$1
    package_name=$2
    debug_mode=$3

    # Colcon build must be run from the workspace directory
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

    # Use sequential executor as parallel builds can use too much memory
    $build_cmd --executor sequential

    # Source the workspace setup.bash to update the environment
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
        set -- "${@/--debug/}"  # Remove --debug from arguments to avoid interference
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
    elif [ -n "$2" ]; then
        # Make sure package exists in onboard workspace
        if ! package_exists "$2" "$ONBOARD_WS"; then
            echo "Error: Package '$2' not found in onboard workspace."
            return 1
        fi

        # Clean a specific package in the onboard workspace
        clean_workspace "$ONBOARD_WS" "$2"
    else
        clean_workspace "$CORE_WS"
        clean_workspace "$ONBOARD_WS"
    fi

elif [ -n "$1" ]; then
    # Make sure package exists in onboard workspace
    if ! package_exists "$1" "$ONBOARD_WS"; then
        echo "Error: Package '$1' not found in onboard workspace."
        return 1
    fi

    # Build a specific package in the onboard workspace
    build_workspace "$ONBOARD_WS" "$1" "$debug_mode"
else
    # Build all packages in both core and onboard workspaces
    build_workspace "$CORE_WS" "" "$debug_mode"
    build_workspace "$ONBOARD_WS" "" "$debug_mode"
fi

# Return to original directory
cd "$original_cwd" || exit