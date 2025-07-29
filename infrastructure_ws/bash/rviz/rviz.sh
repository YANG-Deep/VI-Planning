#!/bin/bash
echo "Start"
user_home=$(eval echo "~")

source $user_home/viplanning_ws/install/setup.bash
rviz2 -d $user_home/viplanning_ws/bash/rviz/panel.rviz

killall -9 grid_future_cells_publisher_pkg

