cmake_minimum_required(VERSION 2.8.3)
project(servicesim_example_python_solution)

find_package(catkin REQUIRED COMPONENTS
   message_generation
   vision_msgs)

add_message_files(
  FILES
  Contour.msg
)

generate_messages(
  DEPENDENCIES
  vision_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  vision_msgs)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/example_solution.py
  src/get_bbox_distance.py
  src/image_processing_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
