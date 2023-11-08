FROM osrf/ros:humble-desktop
MAINTAINER Alexandros Philotheou alefilot@auth.gr

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /ros2_ws/src
COPY ros2_out/ /ros2_ws/src/ros2_out/
COPY 1.ros2run_talker.sh /
COPY 2.ros2run_listener.sh /

RUN cd /ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build &&\
    source /ros2_ws/install/local_setup.bash

WORKDIR /
CMD ["/bin/bash"]
