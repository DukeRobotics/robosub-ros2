#!/bin/bash
# shellcheck disable=SC1091

# Function to remove paths that start with /home/ubuntu/venv from a given variable
remove_venv_paths() {
    local original_path="$1"
    local modified_path=""
    local path_prefix="/home/ubuntu/venv"

    # Split the original path by colons and iterate over each part
    IFS=':' read -ra paths <<< "$original_path"
    for path in "${paths[@]}"; do
        # Add to modified_path if it doesn't start with the specified prefix
        if [[ "$path" != "$path_prefix"* ]]; then

            # If modified_path is not empty, add a colon before adding the path
            if [[ -n "$modified_path" ]]; then
                modified_path="$modified_path:$path"

            # If modified_path is empty, set it to the path
            else
                modified_path="$path"
            fi
        fi
    done

    echo "$modified_path"
}

# If the number of arguments is not 1, print usage
if [ "$#" -ne 1 ]; then
    echo "Usage: source venv.sh {activate|deactivate}"

# If the first argument is "activate", activate the virtual environment
elif [ "$1" == "activate" ]; then
    source /home/ubuntu/venv/bin/activate
    export PYTHONPATH=$PYTHONPATH:/home/ubuntu/venv/lib/python3.12/site-packages

# If the first argument is "deactivate", deactivate the virtual environment
elif [ "$1" == "deactivate" ]; then

    # Deactivate the virtual environment
    deactivate

    # Modify PYTHONPATH to remove paths that start with /home/ubuntu/venv
    PYTHONPATH=$(remove_venv_paths "$PYTHONPATH")
    export PYTHONPATH

    # Modify PATH to remove paths that start with /home/ubuntu/venv
    PATH=$(remove_venv_paths "$PATH")
    export PATH

# If the first argument is not "activate" or "deactivate", print an error message
else
    echo "Invalid argument: $1"
    echo "Usage: source venv.sh {activate|deactivate}"
fi