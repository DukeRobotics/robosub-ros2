#!/bin/bash
# shellcheck disable=SC2034,SC1091

# Check if the first argument is "skip-wsl" and if the script is running in WSL
# This is used by the Dev Container to avoid starting the container via this script when running on Windows
if [[ "$1" == "skip-wsl" && -f /proc/sys/fs/binfmt_misc/WSLInterop ]]; then
    echo "'skip-wsl' is specified and the script is running in WSL. Exiting..."
    exit 0
fi

# Get the path to the directory containing this script, which is the robosub-ros2 repository root
robosub_ros2_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" || exit ; pwd -P )

# Change to the repository root directory
cd "$robosub_ros2_path" || exit

# Load variables from .env file
set -o allexport
if ! source .env; then
    echo "Error: Failed to source .env file."
    exit 1
fi
set +o allexport

# Make sure ENABLE_GIT is set to "true" or "false"
if [ "$ENABLE_GIT" != "true" ] && [ "$ENABLE_GIT" != "false" ]; then
    echo "Error: ENABLE_GIT must be set to 'true' or 'false' in .env"
    exit 1
fi

# If this is not run by a Github Action, make sure ROBOT_NAME is set
if [ "$1" != "--github-action" ] && [ "$2" != "--github-action" ]; then
    if [ -z "$ROBOT_NAME" ]; then
        echo "Error: ROBOT_NAME is not set in .env"
        exit 1

    # Make sure ROBOT_NAME is a valid robot name
    elif ! grep -Fxq "$ROBOT_NAME" "robot/robot_names"; then
        echo "Error: ROBOT_NAME '$ROBOT_NAME' is not a valid robot name."
        exit 1
    fi

# If this is run by a Github Action, set ROBOT_NAME to an empty string
else
    export ROBOT_NAME=""
fi

# Create ~/.foxglove-studio directory if it doesn't exist
# On a Linux host, this ensures that the directory is owned by the user and not root
mkdir -p ~/.foxglove-studio

# Read Git username and email from .env or default to global Git settings
GIT_USER_NAME=$(git config --global user.name)
GIT_USER_EMAIL=$(git config --global user.email)

# Year and week number in the format YYYY-WW
# Invalidate the Docker build cache weekly to ensure consistent images across contributors
year_week=$(date +%Y-%U)

# Command used to build the Docker image
docker_build_cmd="docker build --build-arg CACHE_BUSTER='$year_week' --build-arg ENABLE_GIT='$ENABLE_GIT'"

# If the first or second argument is --no-cache, build the image without cache
if [ "$1" == "--no-cache" ] || [ "$2" == "--no-cache" ]; then
    docker_build_cmd+=" --no-cache"
fi

# Check if USER_UID and USER_GID are set, if not, set them to 1000 by default
USER_UID=${USER_UID:-1000}
USER_GID=${USER_GID:-1000}
docker_build_cmd+=" --build-arg USER_UID=$USER_UID --build-arg USER_GID=$USER_GID"

# If the user wants to set up Git in the container, add the necessary build arguments and secrets
if [ "$ENABLE_GIT" == "true" ]; then
    # List of required environment variables
    required_env_vars=("GITHUB_AUTH_SSH_KEY_PRIV_PATH" "GITHUB_AUTH_SSH_KEY_PUB_PATH" "GITHUB_SIGNING_SSH_KEY_PRIV_PATH" "GIT_ALLOWED_SIGNERS_PATH")

    # Loop through each required environment variable and check if it is set
    for env_var in "${required_env_vars[@]}"; do
        if [ -z "${!env_var}" ]; then
            echo "Error: $env_var is not set in .env"
            exit 1
        fi
    done

    docker_build_cmd+=" --build-arg GIT_USER_NAME='$GIT_USER_NAME'"
    docker_build_cmd+=" --build-arg GIT_USER_EMAIL='$GIT_USER_EMAIL'"
    docker_build_cmd+=" --secret id=github_auth_ssh_key,src='$GITHUB_AUTH_SSH_KEY_PRIV_PATH'"
    docker_build_cmd+=" --secret id=github_auth_ssh_key_pub,src='$GITHUB_AUTH_SSH_KEY_PUB_PATH'"
    docker_build_cmd+=" --secret id=github_signing_ssh_key,src='$GITHUB_SIGNING_SSH_KEY_PRIV_PATH'"
    docker_build_cmd+=" --secret id=git_allowed_signers,src='$GIT_ALLOWED_SIGNERS_PATH'"
fi

# If the user has a .foxgloverc file, add it as a secret
if [[ -f "$FOXGLOVERC_PATH" ]]; then
    docker_build_cmd+=" --secret id=foxgloverc,src='$FOXGLOVERC_PATH'"
fi

# Set the build context to the current (docker) directory
docker_build_cmd+=" -t robosub-ros2:latest ./docker"

# Build the Docker image
eval "$docker_build_cmd"

# If $IS_ROBOT is set to "true", then this script is running on the robot
if [ "$IS_ROBOT" == "true" ]; then
    compose_file="robot/docker-compose.yml"
else
    compose_file="docker-compose.yml"
fi

# Use the selected compose file
docker compose -f "$compose_file" up -d
