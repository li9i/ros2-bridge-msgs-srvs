FROM osrf/ros:foxy-ros1-bridge
SHELL ["/bin/bash", "-c"]

# Make workspace dir
# In the end: this repo would be ours and we would be cloning from it
#RUN mkdir -p /ros2_ws/src && \
    #cd /ros2_ws/src && \
    #git clone --branch foxy https://github.com/ros2/ros1_bridge

COPY ros2_ws/ /ros2_ws

COPY 0.build_bridge.sh /
COPY 1.run_bridge.sh /
RUN chmod +x 0.build_bridge.sh
RUN chmod +x 1.run_bridge.sh


## Source both ROS environments
#RUN source /opt/ros/noetic/setup.bash
#RUN source /opt/ros/foxy/setup.bash

## Build the bridge
#RUN cd /ros2_ws
#RUN colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

WORKDIR /
CMD ["/bin/bash"]
