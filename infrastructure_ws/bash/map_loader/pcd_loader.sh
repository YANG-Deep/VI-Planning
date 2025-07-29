#!/bin/bash
echo "Start"
user_home=$(eval echo "~")

source $user_home/viplanning_ws/install/setup.bash
ros2 launch map_loader pointcloud_map_loader.launch.xml

