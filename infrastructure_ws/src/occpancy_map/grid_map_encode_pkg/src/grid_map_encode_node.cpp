#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/grid_cells.hpp>
#include<bitset>
#include<std_msgs/msg/string.hpp>
#include<unordered_map>
#include<viplanning_msgs/msg/occupancy_traj.hpp>
#include<autoware_auto_planning_msgs/msg/trajectory.hpp>
#include<viplanning_msgs/msg/grid_cells_with_id.hpp>
#include<fstream>



//using GridCells = nav_msgs::msg::GridCells;
using GridCells = viplanning_msgs::msg::GridCellsWithId;

constexpr size_t MAX_TOPICS_PER_GROUP = 5;
constexpr size_t TOTAL_TOPICS = 10;
constexpr size_t GROUP_COUNT = TOTAL_TOPICS / MAX_TOPICS_PER_GROUP;

typedef message_filters::sync_policies::ApproximateTime<
    GridCells, GridCells, GridCells, GridCells, GridCells
> GroupSyncPolicy;

typedef message_filters::Synchronizer<GroupSyncPolicy> GroupSync;


class GridMapEncodeNode : public rclcpp::Node
{
public:
    GridMapEncodeNode() : Node("grid_map_encode_node")
    {
        occupancy_map_publisher=this->create_publisher<std_msgs::msg::String>("/occupancy_map_bin",10);
        occupancy_traj_pub=this->create_publisher<viplanning_msgs::msg::OccupancyTraj>("/occupancy_traj",10);
        traj_sub=this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory"
            ,10,std::bind(&GridMapEncodeNode::traj_callback,this,std::placeholders::_1));
        debug_publisher=this->create_publisher<nav_msgs::msg::GridCells>("/debug",10);
        for (size_t group = 0; group < GROUP_COUNT; ++group) {
            std::vector<std::unique_ptr<message_filters::Subscriber<GridCells>>> group_subscribers;
            for (size_t i = 0; i < MAX_TOPICS_PER_GROUP; ++i) {
                size_t topic_index = group * MAX_TOPICS_PER_GROUP + i;
                std::string topic_name = "/grid_future_cells_" + std::to_string(topic_index)+"_with_id";
                auto subscriber = std::make_unique<message_filters::Subscriber<GridCells>>(this, topic_name);
                group_subscribers.push_back(std::move(subscriber));
            }

            group_syncs_.emplace_back(std::make_shared<GroupSync>(
                GroupSyncPolicy(10),
                *group_subscribers[0], *group_subscribers[1], *group_subscribers[2], *group_subscribers[3], *group_subscribers[4]
            ));

            group_syncs_.back()->registerCallback(std::bind(
                &GridMapEncodeNode::group_callback, this, group,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5
            ));

            group_subscribers_.push_back(std::move(group_subscribers));
        }
    }
    
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


    std::vector<double>runtime;

    std::vector<double>orin_size;
    std::vector<double>encoded_size;

private:
    void traj_callback(autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg){
        if(!is_received){
            // for(int i=50;i<msg->points.size();i++){
            //     traj_msg.points.push_back(msg->points[i]);
            // }
            traj_msg=*msg;
            is_received=true;
        }
        else return;
    }

    void group_callback(size_t group_index,
                        const GridCells::ConstSharedPtr& msg1, const GridCells::ConstSharedPtr& msg2,
                        const GridCells::ConstSharedPtr& msg3, const GridCells::ConstSharedPtr& msg4,
                        const GridCells::ConstSharedPtr& msg5)
    {
        std::vector<GridCells::ConstSharedPtr> group_synced_msgs = {msg1, msg2, msg3, msg4, msg5};
        group_synced_messages_[group_index] = group_synced_msgs;

        bool all_groups_synced = true;
        for (const auto& group_msgs : group_synced_messages_) {
            if (group_msgs.empty()) {
                all_groups_synced = false;
                break;
            }
        }

        if (all_groups_synced) {
            std::vector<GridCells::ConstSharedPtr> all_synced_msgs;
            for (const auto& group_msgs : group_synced_messages_) {
                all_synced_msgs.insert(all_synced_msgs.end(), group_msgs.begin(), group_msgs.end());
            }

            process_all_synced_messages(all_synced_msgs);

            for (auto& group_msgs : group_synced_messages_) {
                group_msgs.clear();
            }
        }
    }

    void process_all_synced_messages(const std::vector<GridCells::ConstSharedPtr>& all_synced_msgs)
    {
        // RCLCPP_INFO(this->get_logger(), "Received synchronized messages from %zu topics.", all_synced_msgs.size());
        // viplanning_msgs::msg::OccupancyTraj occupancy_traj;
        // for(int i=0;i<all_synced_msgs.size();i++) occupancy_traj.occupancy_map_t.push_back(*all_synced_msgs[i]);
        // if(is_received){
        //     occupancy_traj.trajectory_t=traj_msg;
        //     is_received=false;
        //     RCLCPP_INFO(this->get_logger(), "publish occupancy_traj");
        //     occupancy_traj_pub->publish(occupancy_traj);
        // }
        // else return;
        unsigned int occupancy_map_size=0;
        for (size_t i = 0; i < all_synced_msgs.size(); ++i) occupancy_map_size=occupancy_map_size+all_synced_msgs[i]->grid_cells.cells.size();
        if(occupancy_map_size==0){
            RCLCPP_INFO(this->get_logger(), "No occupancy map!");
            return;
        }
        std::unique_ptr<char[]> occupancy_map(new char[5000]);
        auto start_time=this->get_clock()->now();
        auto count=encode(occupancy_map,all_synced_msgs);
        auto msg=to_string_msg(occupancy_map,count);
        auto end_time=this->get_clock()->now();
        if(!msg.data.empty()){
            runtime.push_back((end_time.seconds()-start_time.seconds())*1000.0);
        }
        if(msg.data.size()%4!=0) {
            RCLCPP_INFO(this->get_logger(), "Error encode msg!");
            return;
        }
        auto crc=crc8(msg.data);
        msg.data.push_back(crc);
        occupancy_map_publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publish occupancy map! Size: %d bytes",msg.data.size());
    }

    std_msgs::msg::String to_string_msg(std::unique_ptr<char[]>& occupancy_map,unsigned int size){
        std_msgs::msg::String msg;
        for(int i=0;i<size;i++) msg.data.push_back(occupancy_map[i]);
        return msg;
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

    unsigned long long int encode(std::unique_ptr<char[]>& occupancy_map,const std::vector<GridCells::ConstSharedPtr>& all_synced_msgs){
        double all_grid=0;
        std::map<std::pair<int,int>,std::pair<std::bitset<10>,int>>grid_set;
        for(int i=0;i<all_synced_msgs.size();i++){
            for(int k=0;k<all_synced_msgs[i]->grid_cells.cells.size();k++){
                double x=all_synced_msgs[i]->grid_cells.cells[k].x-0.5;
                double y=all_synced_msgs[i]->grid_cells.cells[k].y-0.5;
                //RCLCPP_INFO(this->get_logger(), "base_x:%lf base_y:%lf",x,y);
                auto itr=grid_set.find({x,y});
                if(itr!=grid_set.end()){
                    auto attribute=itr->second;
                    attribute.first.set(i);
                }
                else{
                    std::bitset<10>t(0);
                    t.set(i);
                    grid_set.insert({{x,y},{t,all_synced_msgs[i]->objects_id.data[k]}});
                }
            }
            all_grid=all_grid+all_synced_msgs[i]->grid_cells.cells.size();
        }
        RCLCPP_INFO(this->get_logger(), "cells size %d",grid_set.size());
        orin_size.push_back(all_grid*2);
        encoded_size.push_back(grid_set.size()*3);
        std::bitset<32>container_grid(0);
        unsigned long long int count=0;
        for(auto itr:grid_set){
            std::bitset<8>container_x(itr.first.first);
            std::bitset<8>container_y(itr.first.second);
            std::bitset<7>container_x_;
            std::bitset<7>container_y_;
            std::bitset<8>container_object_id(itr.second.second);
            //RCLCPP_INFO(this->get_logger(), "orin_x:%d orin_y:%d",itr.first.first,itr.first.second);
            int x,y;
            int display_x,display_y;
            if(container_x.test(7)){
                x=(int)container_x.to_ulong()-0x100;
                x=std::abs(x);
                container_x_=std::bitset<7>(x);
                container_x_=~container_x_;
                for (size_t i = 0; i < 7; ++i) {
                    if (container_x_[i]) {
                        container_x_[i] = false;
                    } else {
                        container_x_[i] = true;
                        break;
                    }
                }
                display_x=(int)container_x_.to_ulong()-0x80;
            }
            else{
                x=(int)container_x.to_ulong();
                container_x_=std::bitset<7>(x);
                display_x=(int)container_x_.to_ulong();
            }
            if(container_y.test(7)){
                y=(int)container_y.to_ulong()-0x100;
                y=std::abs(y);
                container_y_=std::bitset<7>(y);
                container_y_=~container_y_;
                for (size_t i = 0; i < 7; ++i) {
                    if (container_y_[i]) {
                        container_y_[i] = false;
                    } else {
                        container_y_[i] = true;
                        break;
                    }
                }
                display_y=(int)container_y_.to_ulong()-0x80;
            }
            else{
                y=(int)container_y.to_ulong();
                container_y_=std::bitset<7>(y);
                display_y=(int)container_y_.to_ulong();
            }
            //RCLCPP_INFO(this->get_logger(), "x:%d y:%d",display_x,display_y);
            for(int k=0;k<7;k++) container_grid[k]=container_x_[k];
            for(int k=0;k<7;k++) container_grid[k+7]=container_y_[k];
            for(int k=0;k<10;k++) container_grid[k+14]=itr.second.first[k];
            for(int k=0;k<8;k++) container_grid[k+24]=container_object_id[k];
            auto byte_grid=container_grid.to_ulong();
            for(int i=0;i<4;i++) occupancy_map[i+count]=((char*)&byte_grid)[i];
            count=count+4;
        }
        return count;
    }

    std::vector<std::vector<std::unique_ptr<message_filters::Subscriber<GridCells>>>> group_subscribers_;

    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_sub;
    autoware_auto_planning_msgs::msg::Trajectory traj_msg;
    bool is_received=false;
    rclcpp::Publisher<viplanning_msgs::msg::OccupancyTraj>::SharedPtr occupancy_traj_pub;

    std::vector<std::shared_ptr<GroupSync>> group_syncs_;
    std::vector<std::vector<GridCells::ConstSharedPtr>> group_synced_messages_{GROUP_COUNT};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr occupancy_map_publisher;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr debug_publisher;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridMapEncodeNode>();
    rclcpp::spin(node);
    //node->writeVectorToCSV(node->runtime,"/home/wodone/viplanning_ws/test_data/runtimr_data/high_encode.csv");
    // node->writeVectorToCSV(node->orin_size,"/home/wodone/viplanning_ws/test_data/encode_test_data/high_orin.csv");
    // node->writeVectorToCSV(node->encoded_size,"/home/wodone/viplanning_ws/test_data/encode_test_data/high_encode.csv");
    rclcpp::shutdown();
    return 0;
}
