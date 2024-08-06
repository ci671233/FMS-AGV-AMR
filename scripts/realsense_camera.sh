#!/bin/bash
# Check if realsense camera node is running and kill it if it is
rosnode list | grep -E 'realsense2_camera' | xargs -I {} rosnode kill {} || true

# Start realsense camera
source /opt/ros/noetic/setup.bash
roslaunch realsense2_camera rs_camera.launch
