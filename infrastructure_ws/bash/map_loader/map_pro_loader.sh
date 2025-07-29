#!/bin/bash
echo "Start"
user_home=$(eval echo "~")

source $user_home/viplanning_ws/install/setup.bash
ros2 launch map_projection_loader map_projection_loader.launch.xml
