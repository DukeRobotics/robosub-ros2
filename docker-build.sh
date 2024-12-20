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
source .env
set -o allexport

# Read Git username and email from .env or default to global Git settings
GIT_USER_NAME=$(git config --global user.name)
GIT_USER_EMAIL=$(git config --global user.email)

# Year and week number in the format YYYY-WW
# Invalidate the Docker build cache weekly to ensure consistent images across contributors
year_week=$(date +%Y-%U)

# Command used to build the Docker image
docker_build_cmd="docker build --build-arg CACHE_BUSTER='$year_week' --build-arg NO_GIT='$NO_GIT'"

# If the first or second argument is --no-cache, build the image without cache
if [ "$1" == "--no-cache" ] || [ "$2" == "--no-cache" ]; then
    docker_build_cmd+=" --no-cache"
fi

# If the user wants to set up Git in the container, add the necessary build arguments and secrets
if [ "$NO_GIT" != "true" ]; then
    docker_build_cmd+=" --build-arg GIT_USER_NAME='$GIT_USER_NAME'"
    docker_build_cmd+=" --build-arg GIT_USER_EMAIL='$GIT_USER_EMAIL'"
    docker_build_cmd+=" --secret id=github_auth_ssh_key,src='$GITHUB_AUTH_SSH_KEY_PRIV_PATH'"
    docker_build_cmd+=" --secret id=github_auth_ssh_key_pub,src='$GITHUB_AUTH_SSH_KEY_PUB_PATH'"
    docker_build_cmd+=" --secret id=github_signing_ssh_key,src='$GITHUB_SIGNING_SSH_KEY_PRIV_PATH'"
    docker_build_cmd+=" --secret id=git_allowed_signers,src='$GIT_ALLOWED_SIGNERS_PATH'"
fi

# Set the build context to the current (docker) directory
docker_build_cmd+=" -t robosub-ros2:latest ./docker"

# Build the Docker image
eval "$docker_build_cmd"

# Set the compose file name based on $ROBOT_NAME
if [ -n "$ROBOT_NAME" ]; then
    compose_file="docker-compose-robot.yml"
else
    compose_file="docker-compose.yml"
fi

# Use the selected compose file
docker compose -f "$compose_file" up -d
