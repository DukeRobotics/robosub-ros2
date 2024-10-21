#!/bin/bash
# shellcheck disable=SC2034,SC1091

# Check if the first argument is "skip-wsl" and if the script is running in WSL
# This is used by the Dev Container to avoid starting the container via this script when running on Windows
if [[ "$1" == "skip-wsl" && -f /proc/sys/fs/binfmt_misc/WSLInterop ]]; then
    echo "'skip-wsl' is specified and the script is running in WSL. Exiting..."
    exit 0
fi

# Load variables from .env file
set -o allexport
source .env
set -o allexport

# Read Git username and email from .env or default to global Git settings
GIT_USER_NAME=$(git config --global user.name)
GIT_USER_EMAIL=$(git config --global user.email)

# Check if the user wants to set up Git in the container
if [ "$NO_GIT" != "true" ]; then
    # Read SSH key contents using the paths from the .env file
    GITHUB_AUTH_SSH_KEY=$(cat "$GITHUB_AUTH_SSH_KEY_PRIV_PATH")
    GITHUB_AUTH_SSH_KEY_PUB=$(cat "$GITHUB_AUTH_SSH_KEY_PUB_PATH")
    GITHUB_SIGNING_SSH_KEY=$(cat "$GITHUB_SIGNING_SSH_KEY_PRIV_PATH")
    GIT_ALLOWED_SIGNERS=$(cat "$GIT_ALLOWED_SIGNERS_PATH")
fi

# Build the Docker image with the SSH keys passed as build arguments
docker compose -f docker-compose-with-git.yml up -d --build
