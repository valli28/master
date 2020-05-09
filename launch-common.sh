#!/bin/bash

# tell the simulation where our start position is.
# Now that the world has been made, I can no longer change the world
export PX4_HOME_LAT=55.396142
export PX4_HOME_LON=10.388953
export PX4_HOME_ALT=0.0

px4_dir=$(pwd)/Firmware

source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo

roslaunch offb movement.launch