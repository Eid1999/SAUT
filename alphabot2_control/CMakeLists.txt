cmake_minimum_required(VERSION 3.0.2)
project(demo_python)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # Add other ROS packages here
  rospy
  std_msgs
  nav_msgs
  waveshare_alphabot2
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

catkin_package(
  CATKIN_DEPENDS 
  rospy 
  std_msgs
  nav_msgs
  waveshare_alphabot2
)

##########

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
