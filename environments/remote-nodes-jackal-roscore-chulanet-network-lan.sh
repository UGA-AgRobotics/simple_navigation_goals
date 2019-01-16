#!/bin/bash

# Env vars for testing ROS nodes on remote computer at brevard house.

# NOTE: set NODEJS_HOST as computer's IP address to broadcast Rover
# Watch app on the local network.

export NODEJS_HOST=192.168.0.10
export NODEJS_PORT=8000
export ROS_IP=192.168.0.10
export ROS_MASTER_URI=http://192.168.0.85:11311
export ROS_WEBSOCKET_URL=ws://192.168.0.10:9090