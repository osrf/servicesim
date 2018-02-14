cmake_minimum_required(VERSION 2.8.3)
project(servicesim)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

#############
## Install ##
#############

# Install launch files
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
