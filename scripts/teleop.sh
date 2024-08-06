#!/bin/bash
# Check if teleop node is running and kill it if it is
rosnode list | grep -E 'turtlebot3_teleop_keyboard' | xargs -I {} rosnode kill {} || true

# Set TurtleBot model to waffle
export TURTLEBOT3_MODEL=waffle
source /opt/ros/noetic/setup.bash

# Start Teleop
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
