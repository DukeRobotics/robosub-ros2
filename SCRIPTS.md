# Scripts

Documentation for the scripts in the repository root.

## Contents
- [`build.sh`](#buildsh)
- [`docker-build.sh`](#docker-buildsh)
- [`lint.py`](#lintpy)
- [`venv.sh`](#venvsh)

## `build.sh`
The `build.sh` script has several options to build and clean packages. The script must be run with `source` to ensure the environment variables are set in the current shell.
- By default, the script builds all packages in the `core` and `onboard` workspaces.
    ```bash
    source build.sh
    ```
- The script accepts an optional first argument that can be `core`, `onboard`, or the name of a specific package in the `onboard` workspace.
    ```bash
    source build.sh [core|onboard|PACKAGE_NAME] [--debug]
    ```
    - If the first argument is `core`, the script builds all packages in the `core` workspace.
    - If the first argument is `onboard`, the script builds all packages in the `onboard` workspace.
    - If the first argument is the name of a specific package in `onboard`, the script builds only that package.
    - If the `--debug` flag is specified, the script builds the package(s) with debug symbols. This applies only to packages built with CMake.
- The first argument can also be `clean` to remove the `build`, `install`, and `log` directories from the `core` and `onboard` workspaces.
    ```bash
    source build.sh clean [core|onboard]
    ```
    - The optional second argument can be `core` or `onboard`.
    - If the second argument is `core`, the script cleans only the `core` workspace.
    - If the second argument is `onboard`, the script cleans only the `onboard` workspace.

## `docker-build.sh`
The `docker-build.sh` script builds the Docker image and starts the container.
- The script builds the Docker image and starts the container with the necessary configuration to sign commits, pull, and push changes to the remote repository.
    ```bash
    ./docker-build.sh
    ```
- The script accepts an optional argument `skip-wsl` that checks if the script is being run in Windows Subsystem for Linux (WSL). If so, the script does not build the image, nor does it run the container. It simply echoes a message and exits. This is used by the Dev Container to avoid starting the container via this script when running on Windows.
    ```bash
    ./docker-build.sh skip-wsl
    ```
- The script accepts an optional flag `--no-cache` to build the Docker image without using the cache.
    ```bash
    ./docker-build.sh --no-cache
    ```

## `lint.py`
The `lint.py` script lint checks files of supported programming languages. It can be executed with the following command:
```bash
./lint.py [OPTIONS]
```
- `-h, --help`: Show the help message and exit.
- `-p, --path [PATH]`: Path to the directory or file to lint, relative to the current working directory. Must be within the repository. Defaults to the repository root.
- `-l, --languages {bash,cpp,python}`: Language(s) to lint. Defaults to linting all supported languages.
- `-f, --fix`: Automatically fix linting errors where possible. Autofix is supported for Bash, C++, and Python.
- `--print-success`: Print the paths to the files that were successfully linted. This is automatically enabled if `--path` is a file.
- `-o, --output-type {capture,terminal,quiet}`: How to handle the outputs of the linting commands. Default is `terminal`.
    - `capture` captures the output and prints it through this script (useful for CI/CD).
    - `terminal` prints the output as it is generated.
    - `quiet` suppresses the output.
- `-s, --sort`: Sort the output by language.
- `--github-action`: Use GitHub Actions workflow commands in the output.
- `--no-git-tree`: Do not check if files are ignored by git. Instead, lint all files in the repository, including git-ignored files. This is useful when this script is run in a CI/CD environment that does not have the git tree available.

## `venv.sh`
The `venv.sh` script activates and deactivates the Python virtual environment. It must be run with `source` to activate the virtual environment in the current shell.

The script requires exactly one argument, which can be `activate` or `deactivate`.
- To activate the virtual environment, run:
    ```bash
    source venv.sh activate
    ```
- To deactivate the virtual environment, run:
    ```bash
    source venv.sh deactivate
    ```