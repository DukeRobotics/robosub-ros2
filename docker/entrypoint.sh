#!/bin/bash

# Function to handle SIGTERM and SIGINT signals (graceful shutdown)
shutdown() {
    exit 0
}

# Trap SIGTERM and SIGINT signals to run the shutdown function
trap 'shutdown' SIGTERM SIGINT

# Keep the script running
while true; do
  sleep 1
done