#!/bin/bash

cd /root/dev/robosub-ros2 || exit

# Run clang-format and capture the output
find . \( -name '*.cpp' -o -name '*.hpp' -o -name '*.c' -o -name '*.h' \) -print0 | xargs -0 clang-format -style=file --dry-run 2>&1 | while IFS= read -r line; do
    # Extract file path, line number, column number, and message using regex
    if [[ $line =~ ^(.*):([0-9]+):([0-9]+):\ ([a-z]+):\ (.*)$ ]]; then
        file="${BASH_REMATCH[1]}"
        line_number="${BASH_REMATCH[2]}"
        column_number="${BASH_REMATCH[3]}"
        severity="${BASH_REMATCH[4]}"
        message="${BASH_REMATCH[5]}"

        # Map clang-format severity to GitHub workflow command
        case $severity in
            error)
                echo "::error file=${file},line=${line_number},col=${column_number}::${message}"
                ;;
            warning)
                echo "::warning file=${file},line=${line_number},col=${column_number}::${message}"
                ;;
            note)
                echo "::notice file=${file},line=${line_number},col=${column_number}::${message}"
                ;;
            *)
                echo "::error file=${file},line=${line_number},col=${column_number}::${message}"
                ;;
        esac
    fi
done