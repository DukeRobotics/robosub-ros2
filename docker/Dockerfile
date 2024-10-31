FROM osrf/ros:jazzy-desktop-full

# Install packages from apt
# Each package is installed in a separate RUN command so that each package is cached separately in Docker
RUN apt update
RUN apt install -y --no-install-recommends openssh-server
RUN apt install -y --no-install-recommends tini
RUN apt install -y --no-install-recommends snapd
RUN apt install -y --no-install-recommends nano
RUN apt install -y --no-install-recommends python3-colcon-common-extensions
RUN apt install -y --no-install-recommends python3-pip
RUN apt install -y --no-install-recommends python3-serial
RUN apt install -y --no-install-recommends ros-jazzy-robot-localization
RUN apt install -y --no-install-recommends python3-jsonpickle
RUN apt install -y --no-install-recommends python3-transforms3d

# Set no password for root user
RUN passwd -d root

# Modify SSH server configuration to allow SSH into the container without password
RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config
RUN echo "PermitEmptyPasswords yes" >> /etc/ssh/sshd_config
RUN echo "PermitUserEnvironment yes" >> /etc/ssh/sshd_config
RUN mkdir -p /root/.ssh
RUN touch /root/.ssh/environment

# SSH server at port 2202
RUN echo "Port 2202" >> /etc/ssh/sshd_config

# Copy SSH config file
COPY ssh_config /root/.ssh/config

# Create directory for SSH server
RUN mkdir /var/run/sshd
RUN chmod 0755 /var/run/sshd

# Define ARGs for SSH keys and Git information
ARG NO_GIT
ARG GITHUB_AUTH_SSH_KEY
ARG GITHUB_AUTH_SSH_KEY_PUB
ARG GITHUB_SIGNING_SSH_KEY
ARG GIT_ALLOWED_SIGNERS
ARG GIT_USER_NAME
ARG GIT_USER_EMAIL

# Configure Git and SSH keys
RUN if [ "$NO_GIT" != "true" ]; then \
    echo "$GITHUB_AUTH_SSH_KEY" > /root/.ssh/github_auth_key && \
    echo "$GITHUB_AUTH_SSH_KEY_PUB" > /root/.ssh/github_auth_key.pub && \
    echo "$GITHUB_SIGNING_SSH_KEY" > /root/.ssh/github_signing_key && \
    echo "$GIT_ALLOWED_SIGNERS" > /root/.ssh/allowed_signers && \
    chmod 600 /root/.ssh/github_auth_key /root/.ssh/github_signing_key && \
    chmod 644 /root/.ssh/github_auth_key.pub /root/.ssh/allowed_signers && \
    chown root:root /root/.ssh/* && \
    git config --global commit.gpgSign true && \
    git config --global gpg.format ssh && \
    git config --global user.signingkey "/root/.ssh/github_signing_key" && \
    git config --global gpg.ssh.allowedSignersFile "/root/.ssh/allowed_signers" && \
    git config --global user.name "$GIT_USER_NAME" && \
    git config --global user.email "$GIT_USER_EMAIL"; \
    fi

# Create development directory
RUN mkdir -p /root/dev/robosub-ros2

# Use SSH entrypoint script as container's .bash_profile
COPY ssh_entry.sh /root/.bash_profile

# Copy ROS bashrc file into container and source it at end of .bashrc
COPY ros_bashrc.sh /root/ros_bashrc.sh
RUN echo "source /root/ros_bashrc.sh" >> /root/.bashrc

# Copy entrypoint script into container and make it executable
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Run entrypoint script via tini to ensure proper SIGTERM handling
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/entrypoint.sh"]
