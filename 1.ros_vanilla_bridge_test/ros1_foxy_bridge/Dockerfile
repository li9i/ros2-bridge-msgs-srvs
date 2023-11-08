FROM osrf/ros:foxy-ros1-bridge

# Make workspace dir and clone the bridge
RUN mkdir -p /ros2_ws/src && \
    cd /ros2_ws/src && \
    git clone --branch foxy https://github.com/ros2/ros1_bridge

# These need to run upon entry to container
COPY 0.build_bridge.sh /
COPY 1.run_bridge.sh /
RUN chmod +x /0.build_bridge.sh
RUN chmod +x /1.run_bridge.sh

WORKDIR /
CMD ["/bin/bash"]
