cmake_minimum_required(VERSION 2.8.3)
project(servicebot_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  position_controllers
  robot_state_publisher
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES servicebot_control
#  CATKIN_DEPENDS controller_manager joint_state_controller robot_state_publisher
#  DEPENDS system_lib
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
