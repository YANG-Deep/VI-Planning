#!/bin/bash
echo "Start"
user_home=$(eval echo "~")

source $user_home/viplanning_ws/install/setup.bash
ros2 run occupancy_map_extract_pkg occupancy_map_extract_node