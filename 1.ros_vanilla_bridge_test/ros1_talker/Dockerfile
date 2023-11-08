FROM osrf/ros:noetic-desktop
MAINTAINER Alexandros Philotheou alefilot@auth.gr

RUN apt-get update
RUN apt-get install -y sudo apt-utils build-essential g++ git libboost-random-dev curl lsb-release python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-rosdep python3-catkin-tools python3-osrf-pycommon

# Use bash and create user
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN useradd -ms /bin/bash user_r1t
USER user_r1t
WORKDIR /home/user_r1t

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/user_r1t/.bashrc
RUN echo "source /home/user_r1t/catkin_ws/devel/setup.bash" >> /home/user_r1t/.bashrc

RUN rosdep update
RUN mkdir -p /home/user_r1t/catkin_ws/src && \
    cd /home/user_r1t/catkin_ws/src/

COPY ros1_talker/ /home/user_r1t/catkin_ws/src/ros1_talker/

# These need to run upon entry to container
COPY 1.roslaunch.sh /
#RUN chmod +x /0.roscore.sh
#RUN chmod +x /1.roslaunch.sh

RUN cd /home/user_r1t/catkin_ws && \
    export CC=gcc && \
    export CXX=g++ && \
    alias g++='g++ -std=c++11' && \
    alias clang++='clang++ -std=c++11' && \
    source /opt/ros/noetic/setup.bash && \
    catkin build ros1_talker && \
    source /opt/ros/noetic/setup.bash && \
    source /home/user_r1t/catkin_ws/devel/setup.bash


WORKDIR /
ENTRYPOINT source /opt/ros/noetic/setup.bash && roscore

# The next five lines + the entrypoint command will make sure that
# when the container is run the ros1_talker node is roslaunched immediately
#RUN echo "#!/bin/bash" > /home/user_r1t/r1t_launch.sh
#RUN echo "source /opt/ros/noetic/setup.bash" >> /home/user_r1t/r1t_launch.sh
#RUN echo "source /home/user_r1t/catkin_ws/devel/setup.bash" >> /home/user_r1t/r1t_launch.sh
#RUN echo "roslaunch ros1_talker ros1_talker.launch" >> /home/user_r1t/r1t_launch.sh
#RUN chmod +x /home/user_r1t/r1t_launch.sh
#ENTRYPOINT bash /home/user_r1t/r1t_launch.sh
