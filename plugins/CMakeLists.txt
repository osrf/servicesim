cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_AUTOMOC ON)

# Find packages
find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package(Gazebo REQUIRED)

# include appropriate directories
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# TrajectoryActor
add_library(TrajectoryActorPlugin SHARED TrajectoryActorPlugin.cc)
target_link_libraries(TrajectoryActorPlugin ${GAZEBO_LIBRARIES})

# NumberVisual
add_library(NumberVisualPlugin SHARED NumberVisualPlugin.cc)
target_link_libraries(NumberVisualPlugin ${GAZEBO_LIBRARIES})

# CreateActor
add_library(CreateActorPlugin SHARED ${headers_MOC}
  CreateActorPlugin.cc)
target_link_libraries(CreateActorPlugin
  ${GAZEBO_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)
