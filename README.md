# Duke Robotics Club - RoboSub ROS 2

The Duke Robotics Club is a student organization of [Duke University](https://duke.edu) that develops Autonomous Underwater Vehicles (AUVs) for the [RoboSub](https://robosub.org) competition. Check out [our website](https://duke-robotics.com) for more information about our club and projects.

This repository contains all code required for running and testing our AUVs. The code is built on the [Robot Operating System (ROS) 2](https://github.com/ros2) framework. We use the [Jazzy Jalisco](https://docs.ros.org/en/jazzy) distribution of ROS 2.

We are currently in the process of migrating our codebase from ROS 1 to ROS 2. The ROS 1 codebase can be found in the [robosub-ros repository](https://github.com/DukeRobotics/robosub-ros).

## Set Up the Repository and Development Environment

### Prerequisites
1. Docker (required)
    - [Windows](https://docs.docker.com/desktop/install/windows-install/)
    - [Mac](https://docs.docker.com/desktop/install/mac-install/)
    - [Linux](https://docs.docker.com/desktop/install/linux/)
2. Git Bash (required for Windows)
    - [Git for Windows](https://gitforwindows.org)
> [!IMPORTANT]
> If you're using Windows, you **must use Git Bash** to run all commands below. The commands are **not** compatible with PowerShell or Command Prompt.
2. [Visual Studio Code](https://code.visualstudio.com) (recommended)
    - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension (recommended)

### Git Configuration
1. All commits pushed to this repository must be signed.
    - You **must** sign commits with a SSH key. Using GPG keys or other signing methods is **not** supported.
    - Follow the instructions in the [GitHub documentation](https://docs.github.com/en/github/authenticating-to-github/managing-commit-signature-verification) to set up commit signing.
2. Set up allowed signers.
    - This file enables git to distinguish which commits were signed by you and which were signed by others. This file is **required** to set up the Docker container.
    - Create a file called `allowed_signers` in the `~/.ssh` directory.
    - Run the following command to copy your email and _public_ signing key to the `allowed_signers` file:
        ```bash
        echo "$(git config user.email) $(cat PATH_TO_PUBLIC_KEY)" >> ~/.ssh/allowed_signers
        ```
        - Replace `PATH_TO_PUBLIC_KEY` with the path to your _public_ signing key.
        - You can run `git config user.signingkey` to get the path to your _private_ signing key. Typically, the path to the _public_ key is the same as the path to the _private_ key with `.pub` appended to the end.
    - Run the following command to tell git about the `allowed_signers` file:
        ```bash
        git config --global gpg.ssh.allowedSigners ~/.ssh/allowed_signers
        ```
    - In any repo on your computer, you can run `git log --show-signature` and git will show which commits were signed by you.
3. If you're on Windows:
    - Turn off `autocrlf` by running the following command in Git Bash:
        ```bash
        git config --global core.autocrlf false
        ```
    - Set `eol` to `lf` by running the following command in Git Bash:
        ```bash
        git config --global core.eol lf
        ```
    - These settings keep line endings as `\n` only when you clone the repository, allowing bash scripts to run correctly.
4. Clone the repository by running the following command:
    ```bash
    git clone git@github.com:DukeRobotics/robosub-ros2.git
    ```

### Set Up the Dotenv File
Create a file in the root of the repository called `.env` with the following contents. Populate the variables with the appropriate values after the `=` sign.:
```
GITHUB_AUTH_SSH_KEY_PRIV_PATH=
GITHUB_AUTH_SSH_KEY_PUB_PATH=
GITHUB_SIGNING_SSH_KEY_PRIV_PATH=
GIT_ALLOWED_SIGNERS_PATH=
NO_GIT=false
```
> [!IMPORTANT]
> Do **not** include any extraneous whitespace or comments in the `.env` file. Do **not** put spaces around the `=` sign.
- `GITHUB_AUTH_SSH_KEY_PRIV_PATH`: Absolute path to the _private_ SSH key used for authenticating with GitHub.
- `GITHUB_AUTH_SSH_KEY_PUB_PATH`: Absolute path to the _public_ SSH key used for authenticating with GitHub.
- `GITHUB_SIGNING_SSH_KEY_PRIV_PATH`: Absolute path to the _private_ SSH key used for signing commits.
- `GIT_ALLOWED_SIGNERS_PATH`: Absolute path to the allowed signers file.

> [!NOTE]
> If your SSH keys are **not** located in plaintext files on your computer, then do **not** include the variables above in `.env`. Instead, include only `NO_GIT=true`.
>
> This disables git authentication and signing inside the Docker container. This is useful for CI/CD pipelines or if you have your SSH keys stored in encrypted files.

### Set Up the Docker Container
Make sure you have Docker running on your machine. Then, follow the instructions below to set up the Docker container.

#### Using VS Code Dev Containers
If you're using VS Code and have the Dev Containers extension installed:

1. Open the repo in VS Code.
2. **Windows users only:** Open a Git Bash terminal in VS Code and run the following command to start the Docker container (make sure your working directory is the repository root):
    ```bash
    ./docker-build.sh
    ```
2. Open the Command Palette (`Ctrl/Cmd + Shift + P`).
3. Run the `Dev Containers: Reopen in Container` command.
4. Wait for the container to finish building.
5. Now, you're ready to start developing!
    - VS Code is automatically configured with helpful extensions and settings.
    - Any changes you make in the `/root/dev/robosub-ros2` directory in the container are reflected in the repository on your host machine.
    - Any terminals you open in VS Code are automatically connected to the Docker container.
    - To open a file or directory in the container in VS Code, run the following command in the integrated terminal:
        ```bash
        code PATH_TO_FILE_OR_DIRECTORY
        ```
        - Replace `PATH_TO_FILE_OR_DIRECTORY` with the path to the file or directory you want to open in VS Code.
        - If it is a file, the file will open in the current VS Code window.
        - If it is a directory, a new VS Code window will open with the directory as the workspace.
    - If you set `NO_GIT=false` in the `.env` file, you can make signed commits and pull/push changes to remote from within the container.
    - You must build the packages in the container before running the code. See the [Build Packages](#build-packages) section for more information.
6. When you're done, simply close the VS Code window to stop the container.

> [!IMPORTANT]
> **Windows Users:**
>
> Windows users **must** execute the `docker-build.sh` script via Git Bash _prior_ to reopening the VS Code window in the Dev Container. This script sets up the container to allow you to sign commits, pull, and push changes to the remote repository.
>
> Do **not** rebuild the container through the Dev Containers extension in VS Code. This will cause the container to lose the necessary configuration to sign commits. Instead execute `docker-build.sh` _locally_ (outside the container) in Git Bash to rebuild the container.
>
> If you get the error `onboard2 is already in use`, run the following command in Git Bash to remove the existing container:
> ```bash
> docker rm -f onboard2
> ```
> Then, run the `docker-build.sh` script again.

#### Without VS Code Dev Containers
If you're **not** using VS Code or do **not** have the Dev Containers extension installed:
1. Open a new terminal and navigate to the root of the repository.
2. Run the following script to build and start the Docker container:
    ```bash
    ./docker-build.sh
    ```
    - If you set `NO_GIT=true` in the `.env` file, then run the following command instead: `docker compose up -d --build`.
3. Once the container is running, access its shell by running:
    ```bash
    ssh -p 2202 root@localhost
    ```
4. Now, you're ready to start developing!
    - Any changes you make in the repository on your host machine are reflected in the `/root/dev/robosub-ros2` directory in the Docker container.
    - If you set `NO_GIT=false` in the `.env` file, you can make signed commits and pull/push changes to remote from within the container.
    - You must build the packages in the container before running the code. See the [Build Packages](#build-packages) section for more information.
    - To open additional terminals in the container, open a new terminal and run the `ssh` command above.
5. When you're done, open a new terminal, navigate to the root of the repository, and run:
    ```bash
    docker compose down
    ```

## Build Packages
1. Open a terminal in the Docker container.
2. Navigate to the root of the repository `/root/dev/robosub-ros2`.
3. Run the following command to build all packages:
    ```bash
    source build.sh
    ```
    - This command builds all packages in the `core` and `onboard` workspaces.
    - See the [Build Script Options](#build-script-options) section for more options.
4. You are now ready to run the code!

### Build Script Options
The `build.sh` script has several options to build and clean packages.
- By default, the script builds all packages in the `core` and `onboard` workspaces.
    ```bash
    source build.sh
    ```
- The script accepts an optional first argument that can be `core`, `onboard`, or the name of a specific package in the `onboard` workspace.
    ```bash
    source build.sh [core|onboard|PACKAGE_NAME]
    ```
    - If the first argument is `core`, the script builds all packages in the `core` workspace.
    - If the first argument is `onboard`, the script builds all packages in the `onboard` workspace.
    - If the first argument is the name of a specific package in `onboard`, the script builds only that package.
- The first argument can also be `clean` to remove the `build`, `install`, and `log` directories from the `core` and `onboard` workspaces.
    ```bash
    source build.sh clean [core|onboard]
    ```
    - The optional second argument can be `core` or `onboard`.
    - If the second argument is `core`, the script cleans only the `core` workspace.
    - If the second argument is `onboard`, the script cleans only the `onboard` workspace.
