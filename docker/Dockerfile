# Base Docker image for ROS2 Jazzy that supports linux/amd64 and linux/arm64
FROM ros:jazzy

# Dummy argument to force Docker to rebuild the image by invalidating the cache when the argument value changes
# Cache invalidation forces Docker to rerun the `apt update` command, ensuring that the latest `apt` package index is used when building the image
ARG CACHE_BUSTER

# *********************************************************************************************************************************************************************************
# All commands below are run as root
USER root

# Install packages from apt
# Each package is installed in a separate RUN command so that each package is cached separately in Docker
# The commands are ordered alphabetically
RUN apt-get update
RUN apt install -y --no-install-recommends clang
RUN apt install -y --no-install-recommends clang-format
RUN apt install -y --no-install-recommends gdb
RUN apt install -y --no-install-recommends gdbserver
RUN apt install -y --no-install-recommends iputils-ping
RUN apt install -y --no-install-recommends libeigen3-dev
RUN apt install -y --no-install-recommends libyaml-cpp-dev
RUN apt install -y --no-install-recommends nano
RUN apt install -y --no-install-recommends nmap
RUN apt install -y --no-install-recommends python3-colcon-common-extensions
RUN apt install -y --no-install-recommends python3-jsonpickle
RUN apt install -y --no-install-recommends python3-opencv
RUN apt install -y --no-install-recommends python3-pip
RUN apt install -y --no-install-recommends python3-transforms3d
RUN apt install -y --no-install-recommends python3-venv
RUN apt install -y --no-install-recommends ros-jazzy-cv-bridge
RUN apt install -y --no-install-recommends ros-jazzy-foxglove-bridge
RUN apt install -y --no-install-recommends ros-jazzy-robot-localization
RUN apt install -y --no-install-recommends ros-jazzy-tf-transformations
RUN apt install -y --no-install-recommends ros-jazzy-vision-msgs
RUN apt install -y --no-install-recommends shellcheck
RUN apt install -y --no-install-recommends tini
RUN apt install -y --no-install-recommends valgrind

# Build and install OSQP from source
RUN git clone --branch release-0.6.3 --recursive https://github.com/osqp/osqp.git && \
    cd osqp && \
    mkdir build && \
    cd build && \
    cmake -G "Unix Makefiles" .. && \
    cmake --build . --target install && \
    cd ../.. && \
    rm -rf osqp

# Build and install osqp-eigen from source
RUN git clone https://github.com/robotology/osqp-eigen.git && \
    cd osqp-eigen && \
    git checkout 85c37623774c682db396505f0d4ea677040c2557 && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
    make install && \
    cd ../.. && \
    rm -rf osqp-eigen

# Install Foxglove CLI from pre-built binaries based on architecture
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "x86_64" ]; then \
        curl -L https://github.com/foxglove/foxglove-cli/releases/latest/download/foxglove-linux-amd64 -o /usr/local/bin/foxglove; \
    elif [ "$ARCH" = "aarch64" ]; then \
        curl -L https://github.com/foxglove/foxglove-cli/releases/latest/download/foxglove-linux-arm64 -o /usr/local/bin/foxglove; \
    else \
        echo "Unsupported architecture for Foxglove CLI: $ARCH" && exit 1; \
    fi && \
    chmod +x /usr/local/bin/foxglove

# Install Arduino CLI
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh

# Copy entrypoint script into container and make it executable
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set no password for root and ubuntu users
RUN passwd -d root
RUN passwd -d ubuntu

# Set up non-root user with provided UID and GID

# On Linux, any files read or modified in the container will be owned by the UID and GID of the user you are running the container as.
# This ownership rule applies to the host as well. For example, if you are running the container as root, then any files created, read, or modified in the container
# will be owned by root in the container and on the host. Since you typically log into the host as a non-root user, you will not be able to modify these files
# without changing the ownership back to your user. The base image used by this Dockerfile comes with a non-root user named ubuntu. Here, we set the UID and GID
# of the ubuntu user to match the UID and GID of the user on the host machine. This way, any files read or modified in the container will be owned by the same
# user on the host machine, avoiding permissions issues. If the host is not running Linux, the UID and GID of the ubuntu user can be set to any value

# (typically 1000 for both); the host user will still be able to read and modify files in the container.
ARG USER_UID
ARG USER_GID
RUN groupmod -g $USER_GID ubuntu
RUN usermod -u $USER_UID -g $USER_GID ubuntu
RUN chown -R ubuntu:ubuntu /home/ubuntu

# *********************************************************************************************************************************************************************************
# All commands below are run as ubuntu (non-root user)
USER ubuntu

# Suppress login message about using sudo
RUN touch /home/ubuntu/.sudo_as_admin_successful

# Set up Python virtual environment and install packages
# Virtual enviornment inherits system packages to ensure compatibility with ROS
# The commands are ordered alphabetically
RUN python3 -m venv /home/ubuntu/venv --system-site-packages
RUN /home/ubuntu/venv/bin/pip install depthai
RUN /home/ubuntu/venv/bin/pip install GitPython
RUN /home/ubuntu/venv/bin/pip install gputil
RUN /home/ubuntu/venv/bin/pip install pandas
RUN /home/ubuntu/venv/bin/pip install pyserial
RUN /home/ubuntu/venv/bin/pip install python-nmap
RUN /home/ubuntu/venv/bin/pip install ruff
RUN /home/ubuntu/venv/bin/pip install scikit-learn
RUN /home/ubuntu/venv/bin/pip install sympy
RUN /home/ubuntu/venv/bin/pip install mypy

# Install Node.js (Jod)
ENV NVM_DIR=/home/ubuntu/.nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.0/install.sh | bash \
    && . $NVM_DIR/nvm.sh \
    && nvm install --lts=Jod

# Create SSH directory
RUN mkdir -p /home/ubuntu/.ssh

# Copy SSH config file, .gdbinit file, and ROS bashrc file into container
# The COPY command by default sets the owner of the copied files to root, so we need to specify the owner explicitly, despite the USER command above
COPY --chown=ubuntu:ubuntu ssh_config /home/ubuntu/.ssh/config
COPY --chown=ubuntu:ubuntu .gdbinit /home/ubuntu/.gdbinit
COPY --chown=ubuntu:ubuntu ros_bashrc.sh /home/ubuntu/ros_bashrc.sh

# Source ROS bashrc file at the end of .bashrc
RUN echo "source /home/ubuntu/ros_bashrc.sh" >> /home/ubuntu/.bashrc

# Create development directory
RUN mkdir -p /home/ubuntu/robosub-ros2

# Define ARGs for Git information
ARG NO_GIT
ARG GIT_USER_NAME
ARG GIT_USER_EMAIL

# Configure Git and SSH keys
# Secrets are mounted as files in /run/secrets and are owned by root so they can only be read using sudo
RUN --mount=type=secret,id=github_auth_ssh_key,required=false \
    --mount=type=secret,id=github_auth_ssh_key_pub,required=false \
    --mount=type=secret,id=github_signing_ssh_key,required=false \
    --mount=type=secret,id=git_allowed_signers,required=false \
    if [ "$NO_GIT" != "true" ]; then \
        sudo cat /run/secrets/github_auth_ssh_key > /home/ubuntu/.ssh/github_auth_key && \
        sudo cat /run/secrets/github_auth_ssh_key_pub > /home/ubuntu/.ssh/github_auth_key.pub && \
        sudo cat /run/secrets/github_signing_ssh_key > /home/ubuntu/.ssh/github_signing_key && \
        sudo cat /run/secrets/git_allowed_signers > /home/ubuntu/.ssh/allowed_signers && \
        chmod 600 /home/ubuntu/.ssh/github_auth_key /home/ubuntu/.ssh/github_signing_key && \
        chmod 644 /home/ubuntu/.ssh/github_auth_key.pub /home/ubuntu/.ssh/allowed_signers; \
        git config --global commit.gpgSign true && \
        git config --global gpg.format ssh && \
        git config --global user.signingkey "/home/ubuntu/.ssh/github_signing_key" && \
        git config --global gpg.ssh.allowedSignersFile "/home/ubuntu/.ssh/allowed_signers" && \
        git config --global user.name "$GIT_USER_NAME" && \
        git config --global user.email "$GIT_USER_EMAIL" && \
        git config --global --add safe.directory /home/ubuntu/robosub-ros2 && \
        git config --global pull.rebase false; \
    fi

# Copy Foxglove CLI configuration file
RUN --mount=type=secret,id=foxgloverc \
    if [ -f /run/secrets/foxgloverc ]; then \
        sudo cat /run/secrets/foxgloverc > /home/ubuntu/.foxgloverc; \
    fi

# Run entrypoint script via tini to ensure SIGTERM is properly handled and zombie processes are reaped
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/entrypoint.sh"]
