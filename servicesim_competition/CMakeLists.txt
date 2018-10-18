cmake_minimum_required(VERSION 2.8.3)
project(servicesim_competition)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
)

# Gazebo
find_package(gazebo 8 QUIET)
if(NOT gazebo_FOUND)
  find_package(gazebo 9 REQUIRED)
endif()

add_message_files(
  FILES
    ActorNames.msg
    Score.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
    DropOffGuest.srv
    NewTask.srv
    PickUpGuest.srv
    RoomInfo.srv
    TaskInfo.srv
    Drift.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

set(trajectory_actor_plugin_name TrajectoryActorPlugin)
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    roscpp
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
  ${catkin_LIBRARIES}
)
add_dependencies(${follow_actor_plugin_name}
  ${PROJECT_NAME}_generate_messages_cpp
)
install(TARGETS ${follow_actor_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############################
## Collision Actor plugin ##
#############################

# Create the libCollisionActorPlugin.so library.
set(collision_actor_plugin_name CollisionActorPlugin)
add_library(${collision_actor_plugin_name} SHARED
  src/CollisionActorPlugin.cc
)
target_link_libraries(${collision_actor_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
add_dependencies(${collision_actor_plugin_name}
  ${PROJECT_NAME}_generate_messages_cpp
)
install(TARGETS ${collision_actor_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############################
## Attach Model plugin ##
#############################

# Create the libAttachModelPlugin.so library.
set(attach_model_plugin_name AttachModelPlugin)
add_library(${attach_model_plugin_name} SHARED
  src/AttachModelPlugin.cc
)
target_link_libraries(${attach_model_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
add_dependencies(${attach_model_plugin_name}
  ${PROJECT_NAME}_generate_messages_cpp
)
install(TARGETS ${attach_model_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############################
###### Vicinity plugin ######
#############################

# Create the libVicinityPlugin.so library.
set(vicinity_plugin_name VicinityPlugin)
add_library(${vicinity_plugin_name} SHARED
  src/VicinityPlugin.cc
)
target_link_libraries(${vicinity_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${vicinity_plugin_name}
  ${PROJECT_NAME}_generate_messages_cpp
)
install(TARGETS ${vicinity_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

##########################
##      Competition     ##
##########################

# Create the libCompetitionPlugin.so library.
set(competition_plugin_name CompetitionPlugin)
add_library(${competition_plugin_name} SHARED
  src/CompetitionPlugin.cc
  src/Conversions.cc
  src/Checkpoint.cc
  src/CP_DropOff.cc
  src/CP_PickUp.cc
  src/PenaltyChecker.cc
)
target_link_libraries(${competition_plugin_name}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(${competition_plugin_name}
  ${PROJECT_NAME}_generate_messages_cpp
)
install(TARGETS ${competition_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############
## Install ##
#############

install(
  DIRECTORY
    launch
    map
    media
    models
    worlds
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
