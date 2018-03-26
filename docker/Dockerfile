# Dockerfile for servicesim
FROM osrf/ros:kinetic-desktop

RUN apt-get update \
 && apt-get install -y \
    wget \
    lsb-release \
    sudo \
 && apt-get clean

#Install gazebo8 latest version
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update \
 && apt-get install -y \
    gazebo8 \
    libgazebo8-dev \
    ros-kinetic-gazebo8-ros-pkgs \
    ros-kinetic-fake-localization \
    ros-kinetic-gazebo8-ros-control \
    mesa-utils \
    terminator \
    nano \
    ros-kinetic-teleop-twist-keyboard \
    ros-kinetic-joint-state-controller \
    ros-kinetic-effort-controllers \
    ros-kinetic-servicesim* \
    ros-kinetic-servicebot* \
 && apt-get clean

 LABEL com.nvidia.volumes.needed="nvidia_driver"
 ENV PATH /usr/local/nvidia/bin:${PATH}
 ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}


 CMD ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && bash"]
