#!/bin/bash

cd /catkin_ws/
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
