FROM osrf/ros:foxy-ros1-bridge
SHELL ["/bin/bash", "-c"]

# Make ROS 1, ROS 2, and bridge workspaces. All three should be separate when
# packages need custom messages. Source:
# https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst#example-workspace-setup
RUN mkdir -p /catkin_ws/src /ros2_ws/src /bridge_ws/src

#-------------------------------------------------------------------------------
# ROS 1
#-------------------------------------------------------------------------------
# In practice this would be substituted by a git clone of all ROS 1 packages
# in development that require custom messages
COPY ros1_pkgs/. /catkin_ws/src

COPY 01.ros1_build.sh /
COPY 11.ros1_roscore.sh /
COPY 14.ros1_run.sh /
RUN chmod +x 01.ros1_build.sh
RUN chmod +x 11.ros1_roscore.sh
RUN chmod +x 14.ros1_run.sh

#-------------------------------------------------------------------------------
# ROS 2
#-------------------------------------------------------------------------------
# In practice this would be substituted by a git clone of all ROS 2 packages
# in development that require custom messages
COPY ros2_pkgs/. /ros2_ws/src

COPY 02.ros2_build.sh /
COPY 12.ros2_run.sh /
RUN chmod +x 02.ros2_build.sh
RUN chmod +x 12.ros2_run.sh

#-------------------------------------------------------------------------------
# Bridge
#-------------------------------------------------------------------------------
# No need to clone the bridge, she isn't modified any more.
# But if need be this is the way:
# RUN cd /ros2_ws/src && \
# RUN git clone --branch foxy https://github.com/ros2/ros1_bridge
COPY bridge_pkg/. /bridge_ws/src

COPY 03.bridge_build.sh /
COPY 13.bridge_run.sh /
RUN chmod +x 03.bridge_build.sh
RUN chmod +x 13.bridge_run.sh

#-------------------------------------------------------------------------------
# Build all packages (The bridge should be built **after**
# ROS 1 and ROS 2 packages have been themselves built)
RUN ./01.ros1_build.sh
RUN ./02.ros2_build.sh
RUN ./03.bridge_build.sh



#-------------------------------------------------------------------------------
# Entrypoint
# After running this container you should execute the following,
# in order, in separate terminals
# ./11.ros1_roscore.sh
# ./12.ros2_run.sh
# ./13.bridge_run.sh
# ./14.ros1_run.sh
WORKDIR /
CMD ["/bin/bash"]
