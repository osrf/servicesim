cmake_minimum_required(VERSION 2.8.3)
project(servicesim_competition)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

# Gazebo
find_package(gazebo 8.2.0 REQUIRED)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  DropOffGuest.srv
  GetGuestIdentity.srv
  PickUpGuest.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

set(trajectory_actor_plugin_name TrajectoryActorPlugin)
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
  LIBRARIES ${trajectory_actor_plugin_name}
)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

#############################
## Trajectory Actor plugin ##
#############################

# Create the libTrajectoryActorPlugin.so library.
add_library(${trajectory_actor_plugin_name} SHARED
  src/TrajectoryActorPlugin.cc
)
target_link_libraries(${trajectory_actor_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
install(TARGETS ${trajectory_actor_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############################
## Follow Actor plugin ##
#############################

# Create the libFollowActorPlugin.so library.
set(follow_actor_plugin_name FollowActorPlugin)
add_library(${follow_actor_plugin_name} SHARED
  src/FollowActorPlugin.cc
)
target_link_libraries(${follow_actor_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
add_dependencies(${follow_actor_plugin_name}
  servicesim_competition_generate_messages_cpp
)
install(TARGETS ${follow_actor_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#############
## Install ##
#############

install(DIRECTORY launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
