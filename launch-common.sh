#!/bin/bash

# tell the simulation where our start position is.
export PX4_HOME_LAT=55.373068
export PX4_HOME_LON=10.401635
export PX4_HOME_ALT=0.0

px4_dir=$(pwd)/Firmware

source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo

roslaunch offb movement.launch