#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include<bitset>
#include<vector>
#include<unordered_map>
#include <nav_msgs/msg/grid_cells.hpp>
#include <viplanning_msgs/msg/occupancy_traj.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<fstream>

using namespace message_filters;

class OccupancyMapExtractNode : public rclcpp::Node
{
public:
    OccupancyMapExtractNode() : Node("occupancy_map_extract_node"){
        occupancy_map_sub_=this->create_subscription<std_msgs::msg::String>("/occupancy_map_bin",10,
            std::bind(&OccupancyMapExtractNode::callback,this,std::placeholders::_1));
        traj_sub_=this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory",10,
            std::bind(&OccupancyMapExtractNode::callback_traj,this,std::placeholders::_1));
        publisher_occupancy_traj=this->create_publisher<viplanning_msgs::msg::OccupancyTraj>("/occupancy_traj",10);
        // subscription_ = this->create_subscription<std_msgs::msg::String>(
        //     "/occupancy_map_bin", 10, std::bind(&OccupancyMapExtractNode::topic_callback, this, std::placeholders::_1));
        for(int i=0;i<publisher_cells.size();i++) publisher_cells[i]=this->create_publisher<nav_msgs::msg::GridCells>("/extracted_grid_0"+std::to_string(i),10);
        // for(int i=0;i<cells_msg.size();i++){
        //     cells_msg[i].cell_height=1.0;
        //     cells_msg[i].cell_width=1.0;
        //     cells_msg[i].header.frame_id="map";
        //     cells_msg[i].header.stamp=this->get_clock()->now();
        // }
    }

    std::vector<double>runtime;
    void writeVectorToCSV(const std::vector<double>& vec, const std::string& filename) {
        std::ofstream csvFile(filename);
        if (!csvFile.is_open()) {
            std::cerr << "无法打开文件 " << filename << std::endl;
            return;
        }

        for (double num : vec) {
            csvFile << num << std::endl;
        }

        csvFile.close();
    }


private:
    void callback_traj(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr& traj_msg){
        traj=*traj_msg;
    }
    
    char crc8(const std::string& data) {
        char crc = 0x00;
        for (char c : data) {
            crc ^= c;
            for (int i = 0; i < 8; ++i) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }

    void callback(const std_msgs::msg::String::ConstPtr& occupancy_map_bin){
        cells.clear();
        if(occupancy_map_bin->data.empty()){
            RCLCPP_ERROR(this->get_logger(), "Receive empty code!");
            return;
        }
        auto reciveCrc=occupancy_map_bin->data.back();
        std_msgs::msg::String occupancy_map_bin_;
        occupancy_map_bin_.data=occupancy_map_bin->data.substr(0,occupancy_map_bin->data.size()-1);
        auto cacuCrc=crc8(occupancy_map_bin_.data);

        if(reciveCrc!=cacuCrc){
            RCLCPP_ERROR(this->get_logger(), "Receive error code!(crc not pass!) size: %d",occupancy_map_bin->data.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "CRC pass!");

        if(occupancy_map_bin_.data.size()%4!=0) {
            RCLCPP_ERROR(this->get_logger(), "Receive error code!(data size error! size: %d",occupancy_map_bin->data.size());
            return;
        }

         RCLCPP_INFO(this->get_logger(), "Data size pass!");

        std::vector<viplanning_msgs::msg::GridCellsWithId> cells_msg{10};
        for(int i=0;i<cells_msg.size();i++){
            cells_msg[i].grid_cells.header.frame_id="map";
            cells_msg[i].grid_cells.header.stamp=this->get_clock()->now();
            cells_msg[i].grid_cells.cell_width=1.0;
            cells_msg[i].grid_cells.cell_height=1.0;
        }
        // for(int i=0;i<cells_msg.size();i++) cells_msg[i].cells.clear();
        RCLCPP_INFO(this->get_logger(), "Receive bin occupancy map! Size: %d bytes", occupancy_map_bin_.data.size());
        auto start_time=this->get_clock()->now();
        for(int i=0;i<occupancy_map_bin_.data.size();i=i+4){
            std::bitset<8>byte0(occupancy_map_bin_.data[i]);
            std::bitset<8>byte1(occupancy_map_bin_.data[i+1]);
            std::bitset<8>byte2(occupancy_map_bin_.data[i+2]);
            std::bitset<8>byte3(occupancy_map_bin_.data[i+3]);
            std::bitset<7>container_x;
            std::bitset<7>container_y;
            std::bitset<10>container_t;
            std::bitset<8>container_object_id;
            for(int k=0;k<7;k++) container_x[k]=byte0[k];
            container_y[0]=byte0[7];
            for(int k=0;k<6;k++) container_y[k+1]=byte1[k];
            container_t[0]=byte1[6];
            container_t[1]=byte1[7];
            for(int k=0;k<8;k++) container_t[k+2]=byte2[k];
            for(int k=0;k<8;k++) container_object_id[k]=byte3[k];
            int x,y;
            if(container_x.test(6)) x=(int)container_x.to_ulong()-0x80;
            else x=(int)container_x.to_ulong();
            if(container_y.test(6)) y=(int)container_y.to_ulong()-0x80;
            else y=(int)container_y.to_ulong();
            //RCLCPP_INFO(this->get_logger(), "x:%d y:%d",-(int)x,-(int)y);
            //RCLCPP_INFO(this->get_logger(), "x:%d y:%d",x,y);
            cells.push_back({{x,y},{container_t,container_object_id.to_ulong()}});
        }
        for(auto cell:cells){
            for(int i=0;i<cell.second.first.size();i++){
                if(cell.second.first[i]){
                    geometry_msgs::msg::Point point;
                    point.x = static_cast<double>(cell.first.first+0.5);
                    point.y = static_cast<double>(cell.first.second+0.5);
                    point.z = 0.0;
                    cells_msg[i].grid_cells.cells.push_back(point);
                    cells_msg[i].objects_id.data.push_back(cell.second.second);
                }
            }
        }
        auto end_time=this->get_clock()->now();
        runtime.push_back((end_time.seconds()-start_time.seconds())*1000.0);
        for(int i=0;i<publisher_cells.size();i++) publisher_cells[i]->publish(cells_msg[i].grid_cells);
        viplanning_msgs::msg::OccupancyTraj occupancy_traj_msg;
        // occupancy_traj_msg.occupancy_map_t=cells_msg;
        occupancy_traj_msg.trajectory_t=traj;
        occupancy_traj_msg.occupancy_map_t=cells_msg;
        publisher_occupancy_traj->publish(occupancy_traj_msg);
    }

    // std::unique_ptr<message_filters::Subscriber<std_msgs::msg::String>> occupancy_map_sub_;
    // std::unique_ptr<message_filters::Subscriber<autoware_auto_planning_msgs::msg::Trajectory>> traj_sub_;
    // std::unique_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<std_msgs::msg::String,autoware_auto_planning_msgs::msg::Trajectory>>> sync_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr occupancy_map_sub_;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_sub_;

    std::vector<rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr>publisher_cells{10};

    autoware_auto_planning_msgs::msg::Trajectory traj;

    rclcpp::Publisher<viplanning_msgs::msg::OccupancyTraj>::SharedPtr publisher_occupancy_traj;
    std::vector<std::pair<std::pair<double,double>,std::pair<std::bitset<10>,int>>>cells;
    // std::vector<nav_msgs::msg::GridCells>cells_msg{10};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyMapExtractNode>();
    rclcpp::spin(node);
    //node->writeVectorToCSV(node->runtime,"/home/user/viplanning_ws/test_data/runtimr_data/high_extract.csv");
    rclcpp::shutdown();
    return 0;
}
