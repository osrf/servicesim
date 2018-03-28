# Dockerfile for servicesim
FROM osrf/ros:kinetic-desktop

RUN apt-get update \
 && apt-get install -y \
    lsb-release \
    terminator \
    vim \
    wget \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

#Install gazebo8 latest version
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update \
 && apt-get install -y \
    gazebo8 \
    libgazebo8-dev \
    ros-kinetic-gazebo8-ros-pkgs \
    ros-kinetic-gazebo8-ros-control \
    ros-kinetic-servicesim \
    ros-kinetic-servicesim-example-python-solution \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

 LABEL com.nvidia.volumes.needed="nvidia_driver"
 ENV PATH /usr/local/nvidia/bin:${PATH}
 ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

 # setup entrypoint
 COPY ./ros_entrypoint.sh /

 ENTRYPOINT ["/ros_entrypoint.sh"]
 CMD ["bash"]
