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

# Bridge
COPY bridge/bridge_ws/ /bridge_ws
COPY 03.bridge_build.sh /
COPY 13.bridge_run.sh /
RUN chmod +x 03.bridge_build.sh
RUN chmod +x 13.bridge_run.sh

# ROS 1
COPY ros1/catkin_ws/ /catkin_ws
COPY 01.ros1_build.sh /
COPY 11.ros1_roscore.sh /
COPY 14.ros1_run.sh /
RUN chmod +x 01.ros1_build.sh
RUN chmod +x 11.ros1_roscore.sh
RUN chmod +x 14.ros1_run.sh

# ROS 2
COPY ros2/ros2_ws/ /ros2_ws
COPY 02.ros2_build.sh /
COPY 12.ros2_run.sh /
RUN chmod +x 02.ros2_build.sh
RUN chmod +x 12.ros2_run.sh

# Execute this to build everything
COPY 00.build.sh /
RUN chmod +x 00.build.sh

WORKDIR /
CMD ["/bin/bash"]
