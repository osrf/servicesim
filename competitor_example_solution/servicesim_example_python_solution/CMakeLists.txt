cmake_minimum_required(VERSION 2.8.3)
project(servicesim_example_python_solution)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   message_generation
   std_msgs)

add_message_files(
  FILES 
  Contour.msg
)

generate_messages(
 DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  gazebo_msgs
  roscpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/example_solution.py
  src/image_processing_example.py
  src/get_bbox_distance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
