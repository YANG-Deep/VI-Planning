#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <cmath>
#include <casadi/casadi.hpp>
#include<viplanning_msgs/msg/occupancy_traj.hpp>
#include<nav_msgs/msg/path.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<geometry_msgs/msg/point_stamped.hpp>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>
#include<fstream>

using Pose = std::tuple<double, double, double>;

Eigen::Vector3d quaternionRotateVector(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& vector) {

    return quaternion * vector;
}


Eigen::Vector3d calculatePointInYawDirectionQuaternion(const Eigen::Vector3d& position,
                                                       const Eigen::Quaterniond& quaternion,
                                                       const Eigen::Vector3d& localPoint) {
    Eigen::Vector3d rotatedPoint = quaternionRotateVector(quaternion, localPoint);

    Eigen::Vector3d worldPoint = rotatedPoint + position;

    return worldPoint;
}

class CollisionNonlinearOptimizer {
public:
    CollisionNonlinearOptimizer(int trajectory_len, double dt, double sigma, double alpha_collision, const std::vector<std::vector<std::pair<double, double>>>& obj_pixel_pos)
        : dt(dt), trajectory_len(trajectory_len), sigma(sigma), alpha_collision(alpha_collision), obj_pixel_pos(obj_pixel_pos) {
        initOptimization();
    }

    
    void setReferenceTrajectory(const std::vector<Pose>& reference_trajectory) {
        casadi::DM ref_traj(reference_trajectory.size(), 2);
        for (size_t i = 0; i < reference_trajectory.size(); ++i) {
            ref_traj(i, 0) = std::get<0>(reference_trajectory[i]);
            ref_traj(i, 1) = std::get<1>(reference_trajectory[i]);
        }
        optimizer.set_value(ref_traj_param, ref_traj.T());
        setInitialGuess(reference_trajectory);
    }

    void setSolverOptions(const casadi::Dict& options) {
        optimizer.solver("ipopt", options);
    }

    casadi::OptiSol solve() {
        return optimizer.solve();
    }

    double dt;
    int trajectory_len;
    int current_index;
    double sigma;
    double alpha_collision;
    std::vector<std::vector<std::pair<double, double>>> obj_pixel_pos;
    casadi::Opti optimizer;
    casadi::MX state;
    casadi::MX position_x;
    casadi::MX position_y;
    casadi::MX ref_traj_param;

private:
    void initOptimization() {
        int nx = 2;  
        optimizer = casadi::Opti();

        createDecisionVariables();
        createParameters();
        setObjective();

        casadi::Dict solver_options;
        solver_options["ipopt.print_level"] = 0;
        solver_options["print_time"] = 0;
        solver_options["ipopt.sb"] = "yes";
        optimizer.solver("ipopt", solver_options);
    }

    void createDecisionVariables() {
        state = optimizer.variable(2, trajectory_len);
        position_x = state(0, casadi::Slice());
        position_y = state(1, casadi::Slice());
    }

    void createParameters() {
        ref_traj_param = optimizer.parameter(2, trajectory_len);
    }

    void setObjective() {
        double alpha_xy = 1.0;
        casadi::MX cost_stage = alpha_xy * casadi::MX::sumsqr(casadi::sq(ref_traj_param - casadi::MX::vertcat({position_x, position_y})));
        double alpha_collision = this->alpha_collision;
        casadi::MX cost_collision = 0;
        double normalizer = 1 / (2.507 * sigma);

        for (size_t t = 0; t < obj_pixel_pos.size(); ++t) {
            for (size_t i = 0; i < obj_pixel_pos[t].size(); ++i) {
                double col_x = obj_pixel_pos[t][i].first;
                double col_y = obj_pixel_pos[t][i].second;
                cost_collision += alpha_collision * normalizer * casadi::MX::exp(-((position_x(t) - col_x) * (position_x(t) - col_x) + (position_y(t) - col_y) * (position_y(t) - col_y)) / (2 * sigma * sigma));
            }
        }

        optimizer.minimize(cost_stage + cost_collision);
    }

    void setInitialGuess(const std::vector<Pose>& reference_trajectory) {
        casadi::DM initial_guess(reference_trajectory.size(), 2);
        for (size_t i = 0; i < reference_trajectory.size(); ++i) {
            initial_guess(i, 0) = std::get<0>(reference_trajectory[i]);
            initial_guess(i, 1) = std::get<1>(reference_trajectory[i]);
        }
        optimizer.set_initial(state(casadi::Slice(0, 2), casadi::Slice()), initial_guess.T());
    }
};

struct VehicleMotionMetrics {
    std::vector<double> longitudinal_acceleration;
    std::vector<double> lateral_acceleration;
    std::vector<double> longitudinal_jerk;
    std::vector<double> lateral_jerk;
};


class CollisionOptimizationNode : public rclcpp::Node {
public:
    CollisionOptimizationNode() : Node("collision_optimization_node") {
        sub_input=this->create_subscription<viplanning_msgs::msg::OccupancyTraj>("/occupancy_traj",10,
            std::bind(&CollisionOptimizationNode::input_callback,this,std::placeholders::_1));
        sub_pose=this->create_subscription<geometry_msgs::msg::PoseStamped>("/localization/pose_estimator/pose",10,
            std::bind(&CollisionOptimizationNode::pose_callback,this,std::placeholders::_1));
        pub_outputTraj=this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("/output/Traj",10);
        pub_orinPath=this->create_publisher<nav_msgs::msg::Path>("/orin_path",10);
        pub_outputPath=this->create_publisher<nav_msgs::msg::Path>("output_path",10);
        for(int i=0;i<50;i++){
            pub_occmask.push_back(this->create_publisher<nav_msgs::msg::GridCells>("/occmask_"+std::to_string(i),10));
        }
        pub_d0=this->create_publisher<geometry_msgs::msg::PointStamped>("/d0",10);
        pub_d1=this->create_publisher<geometry_msgs::msg::PointStamped>("/d1",10);
        planning_steps = 10;
        bev_h = 60;
        bev_w = 60;
        occ_filter_range = 5.0;
        sigma = 1.0;
        alpha_collision = 5.0;
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

    bool writePairVectorToCSV(const std::vector<std::pair<double, double>>& data, const std::string& filename) {
        std::ofstream csvFile(filename);
        if (!csvFile.is_open()) {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return false;
        }

        csvFile << "x,y" << std::endl;

        for (const auto& pair : data) {
            csvFile << pair.first << "," << pair.second << std::endl;
        }

        csvFile.close();
        return true;
    }

    std::vector<std::pair<double,double>>orin_traj_pose;


private:
    rclcpp::Subscription<viplanning_msgs::msg::OccupancyTraj>::SharedPtr sub_input;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr pub_outputTraj;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_orinPath;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_outputPath;
    std::vector<rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr> pub_occmask;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_d0;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_d1;

    std::vector<geometry_msgs::msg::Point> sdc_traj_all_;
    std::vector<nav_msgs::msg::GridCells> occ_mask_;

    geometry_msgs::msg::PoseStamped pose_msg;

    int planning_steps;
    int bev_h;
    int bev_w;
    double occ_filter_range;
    double sigma;
    double alpha_collision;

    void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg){
        pose_msg=*msg;
        orin_traj_pose.push_back({msg->pose.position.x,msg->pose.position.y});
        RCLCPP_INFO(this->get_logger(),"get pose!");
    }
    
    void input_callback(viplanning_msgs::msg::OccupancyTraj::ConstSharedPtr msg) {
        sdc_traj_all_.clear();
        occ_mask_.clear();
        auto curr_pose=pose_msg.pose;
        int curr_x=curr_pose.position.x;
        int curr_y=curr_pose.position.y;
        double curr_x_d=(double)curr_x+0.5;
        double curr_y_d=(double)curr_y+0.5;
        
        auto current_occ=msg->occupancy_map_t[0];
        
        double d0=1.0;
        double d1=2.0;

        Eigen::Vector3d position(curr_pose.position.x, curr_pose.position.y,curr_pose.position.z);
        Eigen::Quaterniond quaternion(curr_pose.orientation.w,curr_pose.orientation.x,curr_pose.orientation.y,curr_pose.orientation.z);
        Eigen::Vector3d localPoint0(1.0, 0.0, 0.0);
        Eigen::Vector3d localPoint1(2.0, 0.0, 0.0);
        Eigen::Vector3d worldPoint0 = calculatePointInYawDirectionQuaternion(position, quaternion, localPoint0);
        Eigen::Vector3d worldPoint1 = calculatePointInYawDirectionQuaternion(position, quaternion, localPoint1);

        geometry_msgs::msg::Point p_d0;
        geometry_msgs::msg::Point p_d1;
        p_d0.x=worldPoint0.x();
        p_d0.y=worldPoint0.y();
        p_d0.z=worldPoint0.z();
        p_d1.x=worldPoint1.x();
        p_d1.y=worldPoint1.y();
        p_d1.z=worldPoint1.z();
        p_d0.x=std::floor(p_d0.x)+0.5;
        p_d0.y=std::floor(p_d0.y)+0.5;
        p_d1.x=std::floor(p_d1.x)+0.5;
        p_d1.y=std::floor(p_d1.y)+0.5;

        geometry_msgs::msg::PointStamped d0_msg;
        geometry_msgs::msg::PointStamped d1_msg;
        d0_msg.header.frame_id="map";
        d0_msg.header.stamp=this->get_clock()->now();
        d1_msg.header.frame_id="map";
        d1_msg.header.stamp=this->get_clock()->now();
        d0_msg.point=p_d0;
        d1_msg.point=p_d1;

        pub_d0->publish(d0_msg);
        pub_d1->publish(d1_msg);

        int found_index=-1;
        for(int i=0;i<current_occ.grid_cells.cells.size();i++){
            if(current_occ.grid_cells.cells[i].x>=curr_x_d-0.5&&
               current_occ.grid_cells.cells[i].x<=curr_x_d+0.5&&
               current_occ.grid_cells.cells[i].y>=curr_y_d-0.5&&
               current_occ.grid_cells.cells[i].y<=curr_y_d+0.5){
                found_index=i;
                break;
            }
            else continue;
        }
        if(found_index==-1){
            for(int i=0;i<current_occ.grid_cells.cells.size();i++){
                if(current_occ.grid_cells.cells[i].x>=p_d0.x-0.5&&
                   current_occ.grid_cells.cells[i].x<=p_d0.x+0.5&&
                   current_occ.grid_cells.cells[i].y>=p_d0.y-0.5&&
                   current_occ.grid_cells.cells[i].y<=p_d0.y+0.5){
                    found_index=i;
                    break;
                }
                else continue;
            }
            if(found_index==-1){
                for(int i=0;i<current_occ.grid_cells.cells.size();i++){
                    if(current_occ.grid_cells.cells[i].x>=p_d1.x-0.5&&
                       current_occ.grid_cells.cells[i].x<=p_d1.x+0.5&&
                       current_occ.grid_cells.cells[i].y>=p_d1.y-0.5&&
                       current_occ.grid_cells.cells[i].y<=p_d1.y+0.5){
                        found_index=i;
                        break;
                    }
                    else continue;
                }
            }
        }

        if(found_index==-1){
            for(auto it:msg->occupancy_map_t){
                nav_msgs::msg::GridCells processed_occupancy;
                processed_occupancy.header.frame_id="map";
                processed_occupancy.header.stamp=this->get_clock()->now();
                processed_occupancy.cell_height=1.0;
                processed_occupancy.cell_width=1.0;
                processed_occupancy=it.grid_cells;
                RCLCPP_INFO(this->get_logger(),"orin!");
                occ_mask_.push_back(processed_occupancy);
            }
        }
        else{
            auto object_id=current_occ.objects_id.data[found_index];
            for(auto it:msg->occupancy_map_t){
                nav_msgs::msg::GridCells processed_occupancy;
                processed_occupancy.header.frame_id="map";
                processed_occupancy.header.stamp=this->get_clock()->now();
                processed_occupancy.cell_height=1.0;
                processed_occupancy.cell_width=1.0;
                for(int i=0;i<it.objects_id.data.size();i++){
                    if(it.objects_id.data[i]!=object_id) processed_occupancy.cells.push_back(it.grid_cells.cells[i]);
                    else continue;
                }
                RCLCPP_INFO(this->get_logger(),"processed!");
                occ_mask_.push_back(processed_occupancy);
            }
        }
        
        for(int i=0;i<planning_steps;i++){
            pub_occmask[i]->publish(occ_mask_[i]);
        }

        nav_msgs::msg::Path orin_path_msg;
        orin_path_msg.header.frame_id="map";
        orin_path_msg.header.stamp=this->get_clock()->now();

        int start_pose_index=0;
        for(int i=0;i<msg->trajectory_t.points.size();i++){
            if(std::abs(curr_pose.position.x-msg->trajectory_t.points[i].pose.position.x)<=0.5&&
            std::abs(curr_pose.position.y-msg->trajectory_t.points[i].pose.position.y)<=0.5){
                start_pose_index=i;
                break;
            }
            else continue;
        }


        for(int i=40,j=0;j<planning_steps;i++,j++){
            geometry_msgs::msg::Point point;
            point.x=msg->trajectory_t.points[i].pose.position.x;
            point.y=msg->trajectory_t.points[i].pose.position.y;
            point.z=0.0;
            sdc_traj_all_.push_back(point);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id="map";
            pose.header.stamp=this->get_clock()->now();
            pose.pose=msg->trajectory_t.points[i].pose;
            orin_path_msg.poses.push_back(pose);
        }
        pub_orinPath->publish(orin_path_msg);
        performCollisionOptimization(msg->trajectory_t);
        
    }

    void performCollisionOptimization(autoware_auto_planning_msgs::msg::Trajectory orin_traj) {
        std::vector<std::vector<std::pair<double, double>>> pos_xy_t;
        int valid_occupancy_num = 0;
        
        //You need to adjust the size of delay_t according to the communication delay. 
        //For example, if the communication delay is 0.1 seconds, the value of delay_t should be 1.

        int delay_t=0;

        for (int t = delay_t; t < planning_steps; ++t) {
            std::vector<std::pair<double, double>> pos_xy;
            for(int i=0;i<occ_mask_[t].cells.size();i++) pos_xy.emplace_back(occ_mask_[t].cells[i].x,occ_mask_[t].cells[i].y);
            std::vector<std::pair<double, double>> filtered_pos_xy;
            for (const auto& p : pos_xy) {
                double dx = sdc_traj_all_[t].x - p.first;
                double dy = sdc_traj_all_[t].y - p.second;
                if (dx * dx + dy * dy < occ_filter_range * occ_filter_range) {
                    filtered_pos_xy.push_back(p);
                    ++valid_occupancy_num;
                }
            }
            pos_xy_t.push_back(filtered_pos_xy);
        }

        if (valid_occupancy_num == 0) {
            autoware_auto_planning_msgs::msg::Trajectory output_msg;
            output_msg.header.frame_id="map";
            output_msg.header.stamp=this->get_clock()->now();
            for (int k=0;k<sdc_traj_all_.size();k++) {
                autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
                traj_point=orin_traj.points[k];
                traj_point.pose.position.x=sdc_traj_all_[k].x;
                traj_point.pose.position.y=sdc_traj_all_[k].y;
                output_msg.points.push_back(traj_point);
            }
            //pub_outputTraj->publish(output_msg);
            nav_msgs::msg::Path output_path_msg;
            output_path_msg.header.frame_id="map";
            output_path_msg.header.stamp=this->get_clock()->now();
            for(int k=0;k<output_msg.points.size();k++){
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id="map";
                pose.header.stamp=this->get_clock()->now();
                pose.pose=output_msg.points[k].pose;
                output_path_msg.poses.push_back(pose);
            }
            pub_outputPath->publish(output_path_msg);
            std::vector<std::pair<double,double>>output_traj_pose;
            for(auto it:output_path_msg.poses){
                output_traj_pose.push_back({it.pose.position.x,it.pose.position.y});
            }
            //writePairVectorToCSV(output_traj_pose,"/home/user/viplanning_ws/test_data/open_loop_test_data/output_traj_pose_"+std::to_string(this->get_clock()->now().seconds())+".csv");
            //RCLCPP_INFO(this->get_logger(),"publish output traj %d points",output_msg.points.size());
            return;
        }

        CollisionNonlinearOptimizer col_optimizer(planning_steps, 0.5, sigma, alpha_collision, pos_xy_t);
        std::vector<Pose> reference_trajectory;
        for (const auto& point : sdc_traj_all_) {
            reference_trajectory.emplace_back(point.x, point.y, 0.0);
        }
        col_optimizer.setReferenceTrajectory(reference_trajectory);
        auto start_stamp=this->get_clock()->now();
        auto sol = col_optimizer.solve();
        auto end_stamp=this->get_clock()->now();
        runtime.push_back((end_stamp.seconds()-start_stamp.seconds())*1000.0);

        casadi::DM position_x_dm = sol.value(col_optimizer.position_x);
        casadi::DM position_y_dm = sol.value(col_optimizer.position_y);
         
        // RCLCPP_INFO(this->get_logger(),"size1 %d ",position_x_dm.size1());
        // RCLCPP_INFO(this->get_logger(),"size2 %d ",position_x_dm.size2());


        std::vector<double> position_x_values;
        std::vector<double> position_y_values;

        for (int i = 0; i < position_x_dm.size2(); ++i) {
            position_x_values.push_back(position_x_dm(i).operator double());
            position_y_values.push_back(position_y_dm(i).operator double());
        }
        autoware_auto_planning_msgs::msg::Trajectory output_msg;
        output_msg.header.frame_id="map";
        output_msg.header.stamp=this->get_clock()->now();
        for (size_t i = 0; i < position_x_values.size(); ++i) {
            autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
            traj_point=orin_traj.points[i];
            traj_point.pose.position.x=position_x_values[i];
            traj_point.pose.position.y=position_y_values[i];
            output_msg.points.push_back(traj_point);
        }
        nav_msgs::msg::Path output_path_msg;
        output_path_msg.header.frame_id="map";
        output_path_msg.header.stamp=this->get_clock()->now();
        for(int k=0;k<output_msg.points.size();k++){
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id="map";
            pose.header.stamp=this->get_clock()->now();
            pose.pose=output_msg.points[k].pose;
            output_path_msg.poses.push_back(pose);
        }
        pub_outputPath->publish(output_path_msg);
        std::vector<std::pair<double,double>>output_traj_pose;
        for(auto it:output_path_msg.poses){
            output_traj_pose.push_back({it.pose.position.x,it.pose.position.y});
        }
        //writePairVectorToCSV(output_traj_pose,"/home/user/viplanning_ws/test_data/open_loop_test_data/output_traj_pose_"+std::to_string(this->get_clock()->now().seconds())+".csv");
        //pub_outputTraj->publish(output_msg);
        //RCLCPP_INFO(this->get_logger(),"publish output traj %d points",output_msg.points.size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionOptimizationNode>();
    rclcpp::spin(node);
    //node->writePairVectorToCSV(node->orin_traj_pose,"/home/user/viplanning_ws/test_data/open_loop_test_data/orin_traj_pose.csv");
    rclcpp::shutdown();
    return 0;
}
