#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
source catkin_ws/devel/setup.bash
source ros2_ws/install/local_setup.bash
cd bridge_ws/
colcon build --packages-select ros1_bridge --cmake-force-configure
