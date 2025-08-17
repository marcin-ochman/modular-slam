FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"] 
RUN mkdir -p /catkin_ws/src
WORKDIR  /catkin_ws

RUN apt-get update && apt-get install -y git ros-noetic-libg2o ros-noetic-cv-bridge liblapack-dev libblas-dev freeglut3-dev libqglviewer-dev-qt5 libsuitesparse-dev libeigen3-dev tmux

WORKDIR  /catkin_ws/src
RUN git clone https://github.com/tum-vision/lsd_slam.git lsd_slam && cd lsd_slam && git checkout catkin
COPY lsd_slam_working.patch /catkin_ws/src/lsd_slam

WORKDIR /catkin_ws/src/lsd_slam/

RUN git apply lsd_slam_working.patch

WORKDIR  /catkin_ws/

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash;  catkin_make'
