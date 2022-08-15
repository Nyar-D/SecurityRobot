################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(robot_navigation)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################

find_package(catkin REQUIRED
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

################################################################################
# ros server
################################################################################

add_service_files(
  FILES
  MapEditObstacle.srv
  MapCrop.srv
  )
  
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
    
catkin_package()

################################################################################
# Setup for python modules and scripts
################################################################################

catkin_install_python(
  PROGRAMS scripts/robot_map_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# install
################################################################################

install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



# test
# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     std_msgs
#     geometry_msgs
#     message_generation
# )
# include_directories(${catkin_INCLUDE_DIRS})
# add_executable(client src/client.cpp)
# target_link_libraries(client  ${catkin_LIBRARIES})