# Base Docker image for ROS2 Jazzy that supports linux/amd64 and linux/arm64
FROM ros:jazzy

# Dummy argument to force Docker to rebuild the image by invalidating the cache when the argument value changes
# Cache invalidation forces Docker to rerun the `apt update` command, ensuring that the latest `apt` package index is used when building the image
ARG CACHE_BUSTER

# Install packages from apt
# Each package is installed in a separate RUN command so that each package is cached separately in Docker
# The commands are ordered alphabetically
RUN apt update
RUN apt install -y --no-install-recommends clang
RUN apt install -y --no-install-recommends clang-format
RUN apt install -y --no-install-recommends gdb
RUN apt install -y --no-install-recommends gdbserver
RUN apt install -y --no-install-recommends libeigen3-dev
RUN apt install -y --no-install-recommends libyaml-cpp-dev
RUN apt install -y --no-install-recommends nano
RUN apt install -y --no-install-recommends python3-colcon-common-extensions
RUN apt install -y --no-install-recommends python3-pip
RUN apt install -y --no-install-recommends python3-venv
RUN apt install -y --no-install-recommends ros-jazzy-foxglove-bridge
RUN apt install -y --no-install-recommends ros-jazzy-robot-localization
RUN apt install -y --no-install-recommends ros-jazzy-tf-transformations
RUN apt install -y --no-install-recommends shellcheck
RUN apt install -y --no-install-recommends snapd
RUN apt install -y --no-install-recommends tini
RUN apt install -y --no-install-recommends golang
RUN apt install -y --no-install-recommends valgrind

# Set up Python virtual environment and install packages
# Virtual enviornment inherits system packages to ensure compatibility with ROS
# The commands are ordered alphabetically
RUN python3 -m venv /root/dev/venv --system-site-packages
RUN /root/dev/venv/bin/pip install GitPython
RUN /root/dev/venv/bin/pip install pandas
RUN /root/dev/venv/bin/pip install pyserial
RUN /root/dev/venv/bin/pip install ruff
RUN /root/dev/venv/bin/pip install sympy

# Install Node.js (Jod)
ENV NVM_DIR=/root/.nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.0/install.sh | bash \
    && . $NVM_DIR/nvm.sh \
    && nvm install --lts=Jod

# Set no password for root user
RUN passwd -d root

# Create SSH directory
RUN mkdir -p /root/.ssh

# Copy SSH config file
COPY ssh_config /root/.ssh/config

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

# Build and install foxglove CLI from source
RUN git clone https://github.com/foxglove/foxglove-cli.git && \
    cd foxglove-cli && \
    make install && \
    mv ~/go/bin/foxglove /usr/local/bin/foxglove

# Copy .gdbinit file into container
COPY .gdbinit /root/.gdbinit

# Create development directory
RUN mkdir -p /root/dev/robosub-ros2

# Copy ROS bashrc file into container and source it at end of .bashrc
COPY ros_bashrc.sh /root/ros_bashrc.sh
RUN echo "source /root/ros_bashrc.sh" >> /root/.bashrc

# Copy entrypoint script into container and make it executable
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Define ARGs for Git information
ARG NO_GIT
ARG GIT_USER_NAME
ARG GIT_USER_EMAIL

# Configure Git and SSH keys
RUN --mount=type=secret,id=github_auth_ssh_key,required=false \
    --mount=type=secret,id=github_auth_ssh_key_pub,required=false \
    --mount=type=secret,id=github_signing_ssh_key,required=false \
    --mount=type=secret,id=git_allowed_signers,required=false \
    if [ "$NO_GIT" != "true" ]; then \
    cat /run/secrets/github_auth_ssh_key > /root/.ssh/github_auth_key && \
    cat /run/secrets/github_auth_ssh_key_pub > /root/.ssh/github_auth_key.pub && \
    cat /run/secrets/github_signing_ssh_key > /root/.ssh/github_signing_key && \
    cat /run/secrets/git_allowed_signers > /root/.ssh/allowed_signers && \
    chmod 600 /root/.ssh/github_auth_key /root/.ssh/github_signing_key && \
    chmod 644 /root/.ssh/github_auth_key.pub /root/.ssh/allowed_signers; \
    git config --global commit.gpgSign true && \
    git config --global gpg.format ssh && \
    git config --global user.signingkey "/root/.ssh/github_signing_key" && \
    git config --global gpg.ssh.allowedSignersFile "/root/.ssh/allowed_signers" && \
    git config --global user.name "$GIT_USER_NAME" && \
    git config --global user.email "$GIT_USER_EMAIL" && \
    git config --global --add safe.directory /root/dev/robosub-ros2 && \
    git config --global pull.rebase false; \
    fi

# Copy Foxglove CLI configuration file
RUN --mount=type=secret,id=foxgloverc \
    if [ -f /run/secrets/foxgloverc ]; then \
        cat /run/secrets/foxgloverc > ~/.foxgloverc; \
    fi

# Run entrypoint script via tini to ensure SIGTERM is properly handled and zombie processes are reaped
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/entrypoint.sh"]
