FROM osrf/ros:jazzy-desktop-full

# Install packages from apt
# Each package is installed in a separate RUN command so that each package is cached separately in Docker
RUN apt update
RUN apt install -y --no-install-recommends openssh-server
RUN apt install -y --no-install-recommends tini
RUN apt install -y --no-install-recommends snapd
RUN apt install -y --no-install-recommends python3-colcon-common-extensions
RUN apt install -y --no-install-recommends python3-pip
RUN apt install -y --no-install-recommends python3-serial

# Change password for root user to "robotics"
RUN echo root:robotics | chpasswd

# Modifiy SSH server configuration to allow SSH into the container
RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config && \
    echo "PermitUserEnvironment yes" >> /etc/ssh/sshd_config && \
    mkdir -p /root/.ssh && \
    touch /root/.ssh/environment

# SSH server at port 2202
RUN echo "Port 2202" >> /etc/ssh/sshd_config

# Create directory for SSH server
RUN mkdir /var/run/sshd
RUN chmod 0755 /var/run/sshd

# Copy SSH configuration file
COPY ssh_config /root/.ssh/config

RUN mkdir -p /root/dev/robosub-ros2

# Use SSH entrypoint script as container's .bash_profile
COPY ssh_entry.sh /root/.bash_profile

# Copy ROS bashrc file into container and source it at end of .bashrc
COPY ros_bashrc.sh /root/ros_bashrc.sh
RUN echo "source /root/ros_bashrc.sh" >> /root/.bashrc

# Configure git to automatically sign commits with SSH key mounted inside container and to recognize commit signatures
RUN git config --global commit.gpgSign true
RUN git config --global gpg.format ssh
RUN git config --global user.signingkey "/root/.ssh/github_signing_key"
RUN git config --global gpg.ssh.allowedSignersFile "/root/.ssh/allowed_signers"

# Set git user name and email
ARG GIT_USER_NAME
RUN git config --global user.name "$GIT_USER_NAME"

ARG GIT_USER_EMAIL
RUN git config --global user.email "$GIT_USER_EMAIL"

# Copy entrypoint script into container and make it executable
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Run entrypoint script via tini to ensure proper SIGTERM handling
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/entrypoint.sh"]
