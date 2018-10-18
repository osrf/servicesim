cmake_minimum_required(VERSION 2.8.3)
project(servicesim_test)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  servicesim_competition
  servicebot_description
)

find_package(gazebo 8 QUIET)
if(NOT gazebo_FOUND)
  find_package(gazebo 9 REQUIRED)
endif()

catkin_package(
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
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
  target_link_libraries(trajectory_actor_plugin-test
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
  )

  add_rostest_gtest(servicesim_tasks-test
		    servicesim_tasks/servicesim_tasks.test
            	    servicesim_tasks/servicesim_tasks.cpp)
  target_link_libraries(servicesim_tasks-test
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
   )

  if (ENABLE_DISPLAY_TESTS)
  endif()
endif()
