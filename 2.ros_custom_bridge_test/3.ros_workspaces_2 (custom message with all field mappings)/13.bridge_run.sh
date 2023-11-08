#!/bin/bash

source /catkin_ws/install_isolated/setup.bash
source /ros2_ws/install/local_setup.bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
source /bridge_ws/install/local_setup.bash

ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
