cmake_minimum_required(VERSION 2.8.3)
project(servicesim_test)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(gazebo 8.2.0 REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(trajectory_actor_plugin-test
                    trajectory_actor_plugin/trajectory_actor_plugin.test
                    trajectory_actor_plugin/trajectory_actor_plugin.cpp)
  add_dependencies(trajectory_actor_plugin-test
    servicesim_competition
    servicebot_description
  )
  target_link_libraries(trajectory_actor_plugin-test
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
  )

  if (ENABLE_DISPLAY_TESTS)
  endif()
endif()
