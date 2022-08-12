cmake_minimum_required(VERSION 3.0.2)
project(robot_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Vector2.msg
  WheelSpeed.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS scripts/base_controller.py scripts/visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#将/include路径包含到查找头文件路径
include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    )

set( NODE_NAME base_controller )

#将各个文件夹下的所有文件打包
FILE(GLOB ${PROJECT_NAME}_SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")
FILE(GLOB_RECURSE ${PROJECT_NAME}_HEADER FOLLOW_SYMLINKS include/${PROJECT_NAME}/*.h *.hpp)

# # 直接将上述的所有文件进行编译和内部链接
# add_executable(${NODE_NAME}
#   ${${PROJECT_NAME}_SOURCES}
#   ${${PROJECT_NAME}_HEADER}
#   )

# # 链接库
# target_link_libraries(
#   ${NODE_NAME} ${catkin_LIBRARIES} 
#   )
