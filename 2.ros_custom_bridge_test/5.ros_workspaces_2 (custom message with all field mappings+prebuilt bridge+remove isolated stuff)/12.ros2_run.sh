#!/bin/bash

cd ros2_ws/
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 run cpp_pubsub listener
