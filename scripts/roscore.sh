#!/bin/bash
# Check if roscore is running and kill it if it is
pkill -f roscore || true

# Start roscore
source /opt/ros/noetic/setup.bash
roscore
