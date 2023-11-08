#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/user_r1l/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

roscore
