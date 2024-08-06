#!/bin/bash
# Check if SLAM and related nodes are running and kill them if they are
rosnode list | grep -E 'turtlebot3_slam_gmapping|rviz|robot_state_publisher' | xargs -I {} rosnode kill {} || true

# Set TurtleBot model to waffle
export TURTLEBOT3_MODEL=waffle
source /opt/ros/noetic/setup.bash

# Start SLAM and RViz
screen -S slam -dm bash -c "roslaunch turtlebot3_slam turtlebot3_slam.launch"
screen -S rviz -dm bash -c "rosrun rviz rviz -d $(rospack find turtlebot3_slam)/rviz/turtlebot3_slam.rviz"

