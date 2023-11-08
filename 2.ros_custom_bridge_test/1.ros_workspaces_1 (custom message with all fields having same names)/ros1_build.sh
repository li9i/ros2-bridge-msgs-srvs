#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /catkin_ws
catkin_make

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
