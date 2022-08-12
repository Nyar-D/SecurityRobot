#!/bin/bash
export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_MASTER_URI=http://robot::11311
source /opt/ros/noetic/setup.bash
source /opt/ROS_ws/my_robot_ws/devel/setup.bash
export ROS_HOSTNAME=robot
exec "$@"
