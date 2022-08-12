cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  rospy
  geometry_msgs
  message_generation
)

catkin_package()

################################################################################
# Setup for python modules and scripts
################################################################################

catkin_install_python(
  PROGRAMS scripts/odom_ekf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# install
################################################################################

# install(DIRECTORY launch scripts
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

