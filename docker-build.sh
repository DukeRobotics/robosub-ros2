#!/bin/bash

# Load variables from .env file
set -o allexport
source .env
set -o allexport

# Read Git username and email from .env or default to global Git settings
GIT_USER_NAME=$(git config --global user.name)
GIT_USER_EMAIL=$(git config --global user.email)

# Read SSH key contents using the paths from the .env file
GITHUB_AUTH_SSH_KEY=$(cat ${GITHUB_AUTH_SSH_KEY_PRIV_PATH})
GITHUB_AUTH_SSH_KEY_PUB=$(cat ${GITHUB_AUTH_SSH_KEY_PUB_PATH})
GITHUB_SIGNING_SSH_KEY=$(cat ${GITHUB_SIGNING_SSH_KEY_PRIV_PATH})
GIT_ALLOWED_SIGNERS=$(cat ${GIT_ALLOWED_SIGNERS_PATH})

# Build the Docker image with the SSH keys passed as build arguments
docker compose up -d --build
