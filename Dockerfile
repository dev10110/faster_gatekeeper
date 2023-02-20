FROM ros:melodic

RUN apt-get update && \
      apt-get upgrade -y && \
      apt-get install -q -y --no-install-recommends git tmux vim apt-utils && \
      apt-get install -q -y --no-install-recommends wget && \
      apt-get autoremove -y && \
      apt-get clean && \
      rm -rf /var/lib/apt/lists/*

ENV ROS_DIR=/opt/ros/${ROS_DISTRO}

## INSTALL GUROBI
RUN wget https://packages.gurobi.com/9.5/gurobi9.5.2_linux64.tar.gz -P . && \
      tar -xzf gurobi9.5.2_linux64.tar.gz && \
      rm gurobi9.5.2_linux64.tar.gz && \
      mv gurobi952/ /opt && \
      . /opt/ros/melodic/setup.sh 

WORKDIR /opt/gurobi952/linux64/src/build
RUN apt-get update && apt-get install make build-essential -y --no-install-recommends && \
      make && \
      cp libgurobi_c++.a ../../lib/ && \
      . /opt/ros/melodic/setup.sh

## fix gazebo launch error (update mesa to 22.0.2)
RUN apt-get update && apt-get install -y --no-install-recommends \
  software-properties-common 
RUN add-apt-repository ppa:kisak/kisak-mesa -y
RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y --no-install-recommends \
  mesa-utils 

## INSTALL DEPENDENCIES
RUN apt-get update && apt-get install ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-tf2-sensor-msgs -y --no-install-recommends
RUN apt-get update && apt-get install python-pip -y
RUN python -m pip install pyquaternion
RUN apt-get update &&  apt-get install -y --no-install-recommends \ 
  ros-${ROS_DISTRO}-catkin \
  python-catkin-tools  \ 
  ros-${ROS_DISTRO}-pcl-ros  \
  ros-${ROS_DISTRO}-rqt-gui ros-${ROS_DISTRO}-rqt-gui-py   \
  ros-${ROS_DISTRO}-laser-geometry  \
  ros-${ROS_DISTRO}-rviz  \
  ros-${ROS_DISTRO}-tf2-geometry-msgs \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-rqt-common-plugins



## SOURCE FILES
RUN echo source /opt/ros/melodic/setup.bash >> ~/.bashrc
RUN echo source /root/faster_ws/devel/setup.bash >> ~/.bashrc

WORKDIR /root/faster_ws
