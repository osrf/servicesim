# Dockerfile for servicesim
FROM osrf/ros:kinetic-desktop

RUN apt-get update \
    && apt-get install -y \
    lsb-release \
    vim \
    wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install gazebo8
# setup keys
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# setup sources list
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list

# install gazebo8 and required packages
RUN apt-get update \
    && apt-get install -y \
    gazebo8 \
    libgazebo8-dev \
    ros-kinetic-gazebo8-ros-pkgs \
    ros-kinetic-gazebo8-ros-control \
    ros-kinetic-servicesim \
    ros-kinetic-servicesim-example-python-solution \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# setup paths
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

CMD ["bash"]
