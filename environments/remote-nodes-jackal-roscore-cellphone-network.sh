#!/bin/bash

# Env vars for testing jackal on cellphone hotspot at brevard house.
# Nodes are ran on the remote computer, roscore is on Jackal.
export NODEJS_HOST=192.168.43.7
export NODEJS_PORT=8000
export ROS_IP=192.168.43.7
export ROS_MASTER_URI=http://192.168.43.2:11311
export ROS_WEBSOCKET_URL=ws://192.168.43.7:9090