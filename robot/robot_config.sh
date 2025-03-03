#!/bin/bash
# shellcheck disable=SC2139

repo_root="$(dirname "$(dirname "$(realpath "${BASH_SOURCE[0]}")")")"
alias onboard2="${repo_root}/docker-build.sh"
alias bashon2='docker exec -ti -w /home/ubuntu/robosub-ros2 onboard2 bash'
alias dkill='[ "$(docker ps --format {{.Names}})" ] && docker kill $(docker ps --format {{.Names}}) || echo "No Docker containers are running."'
export ROBOSUB_ROS2_COMPOSE_FILE_PATH="${repo_root}/robot/docker-compose.yml"
unset repo_root