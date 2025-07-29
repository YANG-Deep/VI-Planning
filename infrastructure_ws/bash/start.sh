#!/bin/bash
echo "Start VI-planning for infrastructure"
sleep 1s
gnome-terminal -t "map_pro start" -x bash -c "./map_loader/map_pro_loader.sh;exec bash;" 
sleep 1s
gnome-terminal -t "rviz2 start" -x bash -c "./rviz/rviz.sh;exec bash;"
sleep 1s
gnome-terminal -t "track start" -x bash -c "./track_prediction_pipline/track.sh;exec bash;"
sleep 1s
gnome-terminal -t "prediction start" -x bash -c "./track_prediction_pipline/prediction.sh;exec bash;"
sleep 1s
gnome-terminal -t "map_loader start" -x bash -c "./map_loader/map_loader.sh;exec bash;"
sleep 1s
gnome-terminal -t "map_tf start" -x bash -c "./map_loader/map_tf.sh;exec bash;"
sleep 1s
gnome-terminal -t "lidar_center start" -x bash -c "./detection_pipline/lidar_centerpoint.sh;exec bash;"
sleep 1s
gnome-terminal -t "pcd map start" -x bash -c "./map_loader/pcd_loader.sh;exec bash;"
# sleep 1s
# gnome-terminal -t "object_lanelet_filter start" -x bash -c "./detection_pipline/object_lanelet_filter.sh;exec bash;"
sleep 1s
for (( i=0; i<=9; i++ ))
do
    ros2 run grid_future_cells_publisher_pkg grid_future_cells_publisher  --ros-args -p index:=$i &
done
sleep 1s
gnome-terminal -t "grid map encode start" -x bash -c "./occupancy/grid_map_encode.sh;exec bash;"
