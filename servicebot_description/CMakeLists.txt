cmake_minimum_required(VERSION 2.8.3)
project(servicebot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package()

#############
## Install ##
#############

install(
  FILES
    model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    launch
    materials
    meshes
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
