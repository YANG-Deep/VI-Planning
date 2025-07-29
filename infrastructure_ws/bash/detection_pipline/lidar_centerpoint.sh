#!/bin/bash
echo "Start"
user_home=$(eval echo "~")

source $user_home/viplanning_ws/install/setup.bash
ros2 launch lidar_centerpoint lidar_centerpoint.launch.xml

