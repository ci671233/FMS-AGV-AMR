#!/bin/bash
# Check if web_video_server node is running and kill it if it is
rosnode list | grep -E 'web_video_server' | xargs -I {} rosnode kill {} || true

# Start web_video_server
source /opt/ros/noetic/setup.bash
rosrun web_video_server web_video_server
