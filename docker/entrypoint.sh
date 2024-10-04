#!/bin/bash

# Start SSH service in the foreground
/usr/sbin/sshd -D &

# Capture the PID of the SSH process
SSHD_PID=$!

# Function to handle SIGTERM signal (graceful shutdown)
shutdown() {
    kill -TERM $SSHD_PID  # Stop SSH service gracefully
    wait $SSHD_PID        # Wait for SSH to stop
    exit 0
}

# Trap SIGTERM and SIGINT signals to run the shutdown function
trap 'shutdown' SIGTERM SIGINT

# Wait for all background processes (like SSH) to finish
wait