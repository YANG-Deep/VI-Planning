# \[VI-Planning]

## Note

* If you need to deploy VI-Planing on infrastructure, you need to use infrastructure_ws, and if you need to deploy it on vehicles, you need to use vehicle_ws. These two folders are ros2 workspaces.
<br>
* The "map" folder contains the PCD map and vector map required for the experiments. If you need to conduct your own experiments, you need to prepare your own PCD map and vector map. Because we are using the map loading module of autoware.universe, you need to name the PCD map as "pointcloud_map.pcd". Moreover, you need to draw a vector map based on the PCD map, which can be done using Autoware's online map editor (xxx.com). After drawing the vector map, you need to name it "lanelet2_map.osm". The map_projector_info.yaml file does not need any replacement.
<br>
* We do not provide data packages of the original experimental data. If you need to record data packages for experiments, you need to record the original data information of Lidar point clouds on the infrastructure (topic name: "/sensing/lidar/pointcloud"), and record the vehicle's planning trajectory(topic name: "/planning/scenario_planning/trajectory") and self-pose information(topic name: "/localization/pose_estimator/pose") on the vehicle.
