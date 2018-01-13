cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(WanderingActorPlugin)

# Find packages

find_package(Gazebo REQUIRED)

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Create library
add_library(WanderingActorPlugin SHARED WanderingActorPlugin.cc)
target_link_libraries(WanderingActorPlugin ${GAZEBO_LIBRARIES})
