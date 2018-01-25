cmake_minimum_required(VERSION 2.8.3)
project(servicebot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
# 
# install(DIRECTORY urdf
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
# )
# 
# install(DIRECTORY meshes
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
# )
