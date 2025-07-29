#!/bin/bash
echo "Start"
user_home=$(eval echo "~")

source $user_home/viplanning_ws/install/setup.bash
ros2 launch detected_object_validation object_lanelet_filter.launch.xml
