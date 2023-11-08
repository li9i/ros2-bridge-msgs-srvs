#!/bin/bash

source /opt/ros/noetic/setup.bash
source /catkin_ws/install_isolated/setup.bash
source /catkin_ws/devel/setup.bash

roslaunch ros1_talker ros1_talker.launch
