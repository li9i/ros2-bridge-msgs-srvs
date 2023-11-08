FROM osrf/ros:humble-desktop
MAINTAINER Alexandros Philotheou alefilot@auth.gr
SHELL ["/bin/bash", "-c"]

COPY ros2_ws/ /ros2_ws
RUN source /opt/ros/humble/setup.bash

COPY 0.build_packages.sh /
COPY 1.run_packages.sh /
RUN chmod +x 0.build_packages.sh
RUN chmod +x 1.run_packages.sh

WORKDIR /
CMD ["/bin/bash"]
