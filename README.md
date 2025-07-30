# VI-Planning

## Overview

Welcome to **VI-Planning**! This project provides a framework for deploying planning modules on both infrastructure and vehicle platforms using ROS 2 workspaces. It also includes resources and guidelines for conducting experiments with real-world datasets and custom maps.

---

## Getting Started

### Workspace Selection

- **Infrastructure Deployment:**  
  Use the `infrastructure_ws` ROS 2 workspace when deploying VI-Planning modules on infrastructure.
  
- **Vehicle Deployment:**  
  Use the `vehicle_ws` ROS 2 workspace for deployment on vehicles.

---

## Maps

The `map` folder contains all mapping resources necessary for experiments.

- **PCD Map:**  
  - Required file: `pointcloud_map.pcd`
  - Prepare your custom PCD map if conducting your own experiments.
- **Vector Map:**  
  - Required file: `lanelet2_map.osm`
  - Draw your vector map based on the PCD map using Autoware's online map editor ([see tool](xxx.com)).
- **Map Projector Info:**  
  - File: `map_projector_info.yaml`
  - No changes needed for this file.

> **Note:**  
> The map loading module uses [autoware.universe](https://github.com/autowarefoundation/autoware.universe).  
> Ensure your files are named exactly as specified above.

---

## Data Recording Guidelines

To facilitate experiments and benchmarking, record the following topics:

- **On Infrastructure:**  
  - Lidar point cloud data  
    - Topic: `/sensing/lidar/pointcloud`
- **On Vehicle:**  
  - Planning trajectory  
    - Topic: `/planning/scenario_planning/trajectory`
  - Self-pose information  
    - Topic: `/localization/pose_estimator/pose`

---

## Example Dataset

A real-world dataset for a pedestrian dart-out scenario is available, including:

- Infrastructure and vehicle raw sensor data
- Vectorized scenario map

> [Download the dataset here](insert link here)

---

## Additional Notes

- For custom experiments, ensure your maps follow the naming conventions.
- Use the provided tools and topics for seamless integration with Autoware modules.

---

Happy experimenting with VI-Planning!
