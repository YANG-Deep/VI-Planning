#!/bin/bash
echo "Start"
user_home=$(eval echo "~")

source $user_home/viplanning_ws/install/setup.bash
ros2 launch map_tf_generator map_tf_generator.launch.xml
