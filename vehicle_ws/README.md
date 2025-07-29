# \[VI-Planning for vehicle]

## Quick Start

* First, you need to confirm that you have received the occupancy data sent by the infrastructure, and convert the received occupancy data into the ROS 2 topic format for publication. The name of the published topic is "/occupancy_map_bin", and the format is "std_msgs::msgs::String".
<br>
* Next, you can compile the source code. Before compiling the source code, please install the relevant dependencies according to the Artifact Appendix in the paper. You can directly use "colcon build" to compile the source code.
<br>
* Finally, you need to use the autoware.universe autonomous driving system to plan the trajectory, and then run the start.sh script in the bash folder.

