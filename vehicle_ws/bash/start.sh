#!/bin/bash
echo "Start VI-planning for vehicle"
sleep 1s
gnome-terminal -t "occupancy map extract start" -x bash -c "./occupancy/occupancy_map_extract.sh;exec bash;"
sleep 1s
gnome-terminal -t "planning optimization start" -x bash -c "./occupancy/planning_refine.sh;exec bash;"