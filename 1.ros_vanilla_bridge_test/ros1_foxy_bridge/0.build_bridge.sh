#!/bin/bash
# Run this script as soon as the container is up
# With many thanks from:
# https://answers.ros.org/question/363335/building-ros1_bridge-fails-in-docker-container-noetic-foxy/
# and
# https://github.com/ros2/ros1_bridge/blob/master/README.md

# Source both ROS environments
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

# Build the bridge
cd /ros2_ws
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
