# VI-Planning

## Overview

Welcome to **VI-Planning**! VI-Planning is the first planning-oriented and plug-and-play, infrastructure-assisted autonomous driving system. The core idea of VI-Planning is to leverage the historical observations of infrastructure to generate scene-level future occupancy grid maps. These occupancy grid maps can seamlessly integrate with most autonomous driving frameworks (both modularand end-to-end) without requiring architectural changes, enabling direct optimization of planning outcomes. This project provides a guidance for deploying VI-Planning on both infrastructure and vehicle using ROS 2 workspaces. 

<div align="center">

  <table>
    <tr>
      <td align="center">
        <b>Future Occupancy Generation in Infrastructure</b><br>
        <video src="/asset/infrastructure.mp4" width="380" controls loop muted playsinline></video>
      </td>
      <td align="center">
        <b>Planning Optimization in Vehicle</b><br>
        <video src="/asset/vehicle.mp4" width="380" controls loop muted playsinline></video>
      </td>
    </tr>
  </table>

</div>


---

## Getting Started

### Workspace Selection

 <details>
<summary><strong>Infrastructure Deployment Workflow</strong> (Click to expand/collapse)</summary>

Use the `infrastructure_ws` ROS 2 workspace when deploying VI-Planning modules on infrastructure.

1. **Prepare Lidar Raw Data**
    - Provide Lidar point cloud data as raw data, either generated directly by a Lidar driver or from collected point cloud rosbags.
    - Ensure the frequency of the raw data is controlled at **10 Hz**.
2. **Publish Raw Data**
    - Publish the raw data as a ROS2 topic with:
      - Topic name: `/sensing/lidar/pointcloud`
      - `frame_id`: `base_link`
3. **Set Infrastructure2Map Transformation**
    - Determine the relative positional relationship (translation and rotation) between your infrastructure and the map origin point. We suggest using the ICP algorithm.
    - Modify the corresponding parameters in `src/map/map_tf_generator/src/vector_map_tf_generator_node.cpp`.
4. **Install Dependencies and Compile Source Code**
    - Install the relevant dependencies as described in the Artifact Appendix of the paper.
    - Use `colcon build` to compile the source code.
5. **Start the System**
    - Run `start.sh` in the `bash` folder to launch the system.
    - The system will:
      - Take raw data as input
      - Generate detection, tracking, and prediction results
      - Generate occupancy maps (viewable in `rviz2`)
      - Encode occupancy maps and generate occupancy data for the vehicle
6. **Transmit Occupancy Data**
    - Due to differences in RSU devices, only the transmission code for the RSU device used in our experiment is provided.
    - If you need to send occupancy data to the vehicle, please develop your own program to subscribe to the occupancy data generated in the previous step and send it.
    - The encoded occupancy data format is a binary string.

</details>



 <details>
<summary><strong>Vehicle Deployment Workflow</strong> (Click to expand/collapse)</summary>

Use the `vehicle_ws` ROS 2 workspace for deployment  VI-Planning modules on vehicles.

1. **Receive Occupancy Data**
    - Confirm that you have received the occupancy data sent by the infrastructure.
    - Convert the received occupancy data into a ROS 2 topic for publication.
    - The published topic should be named `/occupancy_map_bin` and use the `std_msgs::msg::String` format.
2. **Install Dependencies and Compile Source Code**
    - Install the relevant dependencies as described in the Artifact Appendix of the paper.
    - Use `colcon build` to compile the source code.
3. **Trajectory Planning and System Launch**
    - Use the autoware.universe autonomous driving system to plan the trajectory.
    - Run the `start.sh` script in the `bash` folder.

</details>



---

## Example Dataset

A real-world dataset for a pedestrian dart-out scenario is available, including:

- **Infrastructure raw sensor data:**  
  - Including: raw point cloud data
- **Vehicle raw sensor data:**  
  - Including: raw point cloud data, ego pose, original trajectory
- **PCD Map:**  
  - Required file: `pointcloud_map.pcd`
  - Prepare your custom PCD map if conducting your own experiments. We suggest using the LIO-SAM algorithm to construct your own PCD Map.
- **Vector Map:**  
  - Required file: `lanelet2_map.osm`
  - Draw your vector map based on the PCD map using Autoware's online map editor ([see tool](xxx.com)).

[Download the dataset here](insert link here)

> **Note:**  
> The map loading module uses [autoware.universe](https://github.com/autowarefoundation/autoware.universe).  
> Ensure your files are named exactly as specified above.


---

## Data Recording Guidelines

If you want to run VI-Planning on your own scenario, please record the following topic:

- **On Infrastructure:**  
  - Lidar point cloud data  
    - Topic: `/sensing/lidar/pointcloud`
- **On Vehicle:**  
  - Planning trajectory  
    - Topic: `/planning/scenario_planning/trajectory`
  - Self-pose information  
    - Topic: `/localization/pose_estimator/pose`

---

## Additional Notes

- For custom experiments, ensure your maps follow the naming conventions.
- Use the provided tools and topics for seamless integration with Autoware modules.

---

Happy experimenting with VI-Planning!
