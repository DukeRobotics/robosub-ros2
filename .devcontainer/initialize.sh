#!/bin/bash

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P) || exit 1
build_args=()

# initializeCommand has no rebuild event variable. On Linux, read the arguments of
# the ancestor Dev Containers CLI process, which runs this hook before removing
# the existing container. Keep this in sync with the CLI's up options:
# https://github.com/devcontainers/cli/blob/main/src/spec-node/devContainersSpecCLI.ts
launcher_pid=$PPID
while [[ "$launcher_pid" -gt 1 && -r "/proc/$launcher_pid/cmdline" && -r "/proc/$launcher_pid/status" ]]; do
    launcher_args=()
    is_devcontainer_cli=false
    while IFS= read -r -d '' arg; do
        launcher_args+=("$arg")
        if [[ "$arg" == */devContainersSpecCLI.js ]]; then
            is_devcontainer_cli=true
        fi
    done < "/proc/$launcher_pid/cmdline"

    if [[ "$is_devcontainer_cli" == "true" ]]; then
        for ((index = 0; index < ${#launcher_args[@]}; index++)); do
            arg=${launcher_args[$index]}
            next_arg=${launcher_args[$((index + 1))]}
            case "$arg" in
                --remove-existing-container|--remove-existing-container=true)
                    if [[ "$next_arg" != "false" ]]; then
                        build_args+=(--rebuild)
                    fi
                    ;;
                --build-no-cache|--build-no-cache=true)
                    if [[ "$next_arg" != "false" ]]; then
                        build_args+=(--no-cache)
                    fi
                    ;;
            esac
        done
        break
    fi

    parent_pid=0
    while read -r key value; do
        if [[ "$key" == "PPid:" ]]; then
            parent_pid=$value
            break
        fi
    done < "/proc/$launcher_pid/status"
    launcher_pid=$parent_pid
done

exec bash "$repo_root/docker-build.sh" skip-wsl --devcontainer "${build_args[@]}"
