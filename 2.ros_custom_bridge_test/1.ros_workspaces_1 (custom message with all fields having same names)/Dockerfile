FROM osrf/ros:foxy-ros1-bridge
SHELL ["/bin/bash", "-c"]

# Make workspace dir
# In the end: this repo (or these---meaning the bridge, ros1, and ros2
# subdirectories---repos) would be ours and we would be cloning it
#RUN mkdir -p /ros2_ws/src && \
    #cd /ros2_ws/src && \
    #git clone --branch foxy https://github.com/ros2/ros1_bridge

# Copy ROS 1, ROS 2, and bridge workspaces. All three should be separate when
# packages need custom messages. Source:
# https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst#example-workspace-setup

RUN mkdir /bridge_ws /catkin_ws /ros2_ws

# ROS 1
COPY ros1/catkin_ws/ /catkin_ws
COPY ros1_build.sh /
COPY ros1_run.sh /
RUN chmod +x ros1_build.sh
RUN chmod +x ros1_run.sh

# ROS 2
COPY ros2/ros2_ws/ /ros2_ws
COPY ros2_build.sh /
COPY ros2_run.sh /
RUN chmod +x ros2_build.sh
RUN chmod +x ros2_run.sh

# Bridge
COPY bridge/bridge_ws/ /bridge_ws
COPY bridge_build.sh /
COPY bridge_run.sh /
RUN chmod +x bridge_build.sh
RUN chmod +x bridge_run.sh

COPY 0.build.sh /
RUN chmod +x 0.build.sh


WORKDIR /
CMD ["/bin/bash"]
