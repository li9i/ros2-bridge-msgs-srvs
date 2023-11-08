#!/bin/bash
# Run this script as soon as the container is up
source /opt/ros/foxy/setup.bash

# Build the packages
cd /ros2_ws
colcon build --symlink-install --cmake-force-configure

source install/local_setup.bash
