
# Set Up the Repository and Development Environment

Setting up the repository and development environment is an involved process. This document outlines the steps required to set up the repository and development environment on your local machine or robot.

Steps 1-3 need to be completed once to set up the repository and required software. Step 4 needs to be completed each time you want to start developing.

## Steps
1. [Prerequisites](#prerequisites)
2. [Git Configuration](#git-configuration)
3. [Set Up the Dotenv File](#set-up-the-dotenv-file)
4. [Set Up the Docker Container](#set-up-the-docker-container)
    - [Using VS Code Dev Containers](#using-vs-code-dev-containers)
    - [Without VS Code Dev Containers](#without-vs-code-dev-containers)
5. [Set Up Foxglove (Optional)](#set-up-foxglove-optional)
    - [Set Up Foxglove Desktop](#set-up-foxglove-desktop)
    - [Set Up the `.foxgloverc` File](#set-up-the-foxgloverc-file)

## Prerequisites
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

## Git Configuration
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

## Set Up the Dotenv File
Create a file in the root of the repository called `.env` with the following contents. Populate the variables with the appropriate values after the `=` sign.
```
GITHUB_AUTH_SSH_KEY_PRIV_PATH=
GITHUB_AUTH_SSH_KEY_PUB_PATH=
GITHUB_SIGNING_SSH_KEY_PRIV_PATH=
GIT_ALLOWED_SIGNERS_PATH=
NO_GIT=false
ROBOT_NAME=
IS_ROBOT=
FOXGLOVERC_PATH=
USER_UID=
USER_GID=
```
> [!IMPORTANT]
> Do **not** include any extraneous whitespace or comments in the `.env` file. Do **not** put spaces around the `=` sign.
- `GITHUB_AUTH_SSH_KEY_PRIV_PATH`: Absolute path to the _private_ SSH key used for authenticating with GitHub.
- `GITHUB_AUTH_SSH_KEY_PUB_PATH`: Absolute path to the _public_ SSH key used for authenticating with GitHub.
- `GITHUB_SIGNING_SSH_KEY_PRIV_PATH`: Absolute path to the _private_ SSH key used for signing commits.
- `GIT_ALLOWED_SIGNERS_PATH`: Absolute path to the allowed signers file.
- `NO_GIT`: For most users, this should be set to `false`. Set to `true` if you do **not** want to use git inside the Docker container. This is useful for CI/CD pipelines or if you have your SSH keys stored in encrypted files.
    > [!NOTE]
    > If you set `NO_GIT=true`, do not include the variables `GITHUB_AUTH_SSH_KEY_PRIV_PATH`, `GITHUB_AUTH_SSH_KEY_PUB_PATH`, `GITHUB_SIGNING_SSH_KEY_PRIV_PATH`, or `GIT_ALLOWED_SIGNERS_PATH` in `.env`. These variables are only used if `NO_GIT=false`.
- `ROBOT_NAME`: The name of the robot you are developing for. This is used to set the `$ROBOT_NAME` environment variable in the Docker container, which is used by some scripts to determine which robot-specific configuration to use. If you are setting up the repository on the robot, then this should be the name of the robot. If you are setting up the repository on your development machine, then this name can be changed to test different robot configurations.
- `IS_ROBOT` (Optional): Set to `true` if you are setting up the repository on the robot. Set to `false` or do not include this variable in `.env` if you are setting up the repository on your development machine.
- `FOXGLOVERC_PATH` (Optional): Absolute path to the [`.foxgloverc`](#set-up-the-foxgloverc-file) file.
- `USER_UID` and `USER_GID` (Optional): The user ID and group ID of the `ubuntu` user in the Docker container.

    If you are a Linux user, set these variables to your user ID and group ID. This ensures that all files in this repository are owned by that combination of user ID and group ID in both the container and host machine, which avoids file permission issues.

    To retrieve these values, open a terminal outside of the Docker container. To get your user ID, run `id -u`. To get your group ID, run `id -g`.
    > [!NOTE]
    > If you are a Mac or Windows user, do **_not_** include `USER_UID` and `USER_GID` in your `.env` file. Include these variables **_only_** if you are a Linux user. If the variables are not included in `.env`, then the UID and GID will both be set to `1000`, which is the default UID and GID for non-root users in Ubuntu.

## Robot Only Setup
If you are setting up the repository on the robot, there's a few additional steps you need to take. If you are setting up the repository on your development machine, you can skip to the [Set Up the Docker Container](#set-up-the-docker-container) section.

### Set Up Udev Rules
Set up the udev rules to symlink some USB devices to ports that are consistent across reboots and used by code in this repository.

1. Open a terminal outside of the Docker container and navigate to the root of the repository.
2. Run the following command to symlink the `99-robosub-ros2.rules` file located in the repository root to the `/etc/udev/rules.d` directory:
    ```bash
    sudo ln -s $(pwd)/99-robosub-ros2.rules /etc/udev/rules.d/99-robosub-ros2.rules
    ```
3. Run the following command to reload the udev rules:
    ```bash
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
4. To check if the rules were applied correctly, run the following command
    ```bash
    ls /dev
    ```
    and check if the symlinks are present. If the symlinks are not present, reboot the robot and check again.

### Set Up Bash Aliases
Set up bash aliases to make it easier to run common commands associated with this repository _outside_ the Docker container (aliases used _inside_ the Docker container are defined in `docker/ros_bashrc.sh`).

1. Open the `~/.bashrc` file in a text editor.
2. Check if the `~/.bash_aliases` file is sourced in the `~/.bashrc` file. If it is not, add the following line to the `~/.bashrc` file
    ```bash
    source ~/.bash_aliases
    ```
    and create a `~/.bash_aliases` file.
3. Add the following line to the `~/.bash_aliases` file to source the `robot_aliases.sh` file in the repository:
    ```bash
    source /absolute/path/to/robosub-ros2/robot_aliases.sh
    ```
    - Replace `/path/to/robosub-ros2` with the absolute path to the `robosub-ros2` repository on the robot.
4. Source the `~/.bashrc` file by running the following command to apply the changes:
    ```bash
    source ~/.bashrc
    ```
5. Run the following command to check if the aliases were set up correctly:
    ```bash
    alias
    ```
    - You should see a list of all aliases set up on the robot. This list should include the aliases defined in the `robot_aliases.sh` file.
    - If the aliases are not present, check the `~/.bash_aliases` file and the `~/.bashrc` file to ensure they are set up correctly.

## Set Up the Docker Container
The Docker container is used to develop and run the code in a consistent environment. It is configured with all the necessary dependencies and tools to develop the code.

Make sure you have Docker running on your machine. Then, follow the instructions below to set up the Docker container.

> [!NOTE]
> Starting the Docker container will create an empty `~/.foxglove-studio/` directory on your local machine if it does not already exist. Foxglove Desktop uses this directory to load locally installed extensions.

### Using VS Code Dev Containers
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
    - Any changes you make in the `/home/ubuntu/robosub-ros2` directory in the container are reflected in the repository on your host machine.
    - Any terminals you open in VS Code are automatically connected to the Docker container.
    - To open a file or directory in the container in VS Code, run the following command in the integrated terminal:
        ```bash
        code PATH_TO_FILE_OR_DIRECTORY
        ```
        - Replace `PATH_TO_FILE_OR_DIRECTORY` with the path to the file or directory you want to open in VS Code.
        - If it is a file, the file will open in the current VS Code window.
        - If it is a directory, a new VS Code window will open with the directory as the workspace.
    - If you set `NO_GIT=false` in the `.env` file, you can make signed commits and pull/push changes to remote from within the container.
    - You must build the packages in the container before running the code. See [Build Packages in README.md](README.md#build-packages) section for more information.
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

### Without VS Code Dev Containers
If you're **not** using VS Code or do **not** have the Dev Containers extension installed:
1. Open a new terminal and navigate to the root of the repository.
2. Run the following script to build and start the Docker container:
    ```bash
    ./docker-build.sh
    ```
3. Once the container is running, access its shell by running:
    ```bash
    docker exec -ti -w /home/ubuntu/robosub-ros2 onboard2 bash
    ```
    - If you're running this command in Git Bash on Windows, add an extra `/` before `/home`:
        ```bash
        docker exec -ti -w //home/ubuntu/robosub-ros2 onboard2 bash
        ```
4. Now, you're ready to start developing!
    - Any changes you make in the repository on your host machine are reflected in the `/home/ubuntu/robosub-ros2` directory in the Docker container.
    - If you set `NO_GIT=false` in the `.env` file, you can make signed commits and pull/push changes to remote from within the container.
    - You must build the packages in the container before running the code. See the [Build Packages in README.md](README.md#build-packages) section for more information.
    - To open additional terminals in the container, open a new terminal and run the `docker exec` command above.
5. When you're done, open a new terminal _on the host machine_, navigate to the root of the repository, and run:
    ```bash
    docker compose down
    ```

## Set Up Foxglove (Optional)
Follow the following instructions if you plan to develop in the `foxglove` monorepo.

### Set Up Foxglove Desktop
To debug local Foxglove extensions, you must install Foxglove desktop.

1. Download and install [Foxglove desktop](https://foxglove.dev/download) and sign in.

### Set Up the `.foxgloverc` File

Setting up the `.foxgloverc` file with allow you to publish custom Foxglove extensions to your organization.
> [!NOTE]
> Setting up the `.foxgloverc` file requires admin privileges in your Foxglove organization.
1. Go to the [Foxglove settings page](https://app.foxglove.dev/duke-robotics/settings/apikeys) and generate a new API key.

2. Outside of the Docker container, create a `~/.foxgloverc` file with the following information:
```
auth_type: 2
base_url: https://api.foxglove.dev
bearer_token:
```
Add the API key generated in Step 1 as the `bearer_token`.

3. Rebuild the Docker container and run `foxglove auth info`. You should see a message indicating that you have successfully authenticated with an API key.
