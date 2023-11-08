#!/bin/bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

roslaunch ros1_talker ros1_talker.launch
