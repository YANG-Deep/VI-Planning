#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/point.hpp>
#include<std_msgs/msg/int8_multi_array.hpp>
#include<viplanning_msgs/msg/grid_cells_with_id.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<fstream>

double left_bound = -60;
double right_bound = 60;
double up_bound = -60;
double down_bound = 60;
double cell_size = 1.0;
unsigned int update_rate = 200; 
unsigned int future_frame = 10; 

struct Quaternion {
    double w, x, y, z;
    Quaternion(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0)
        : w(w), x(x), y(y), z(z) {}
};

struct EulerAngles {
    double roll, pitch, yaw;
    EulerAngles(double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
        : roll(roll), pitch(pitch), yaw(yaw) {}
};

struct Point2D {
    double x;
    double y;
    Point2D(double x = 0, double y = 0) : x(x), y(y) {}
};

EulerAngles quaternionToEuler(const Quaternion& q) {
    EulerAngles angles;

    angles.roll = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));

    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); 
    else
        angles.pitch = std::asin(sinp);

    angles.yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

    return angles;
}

std::vector<Point2D> get_2d_bbox_vertices(const Point2D& center, double length, double width, double angle) {
    double half_length = length / 2;
    double half_width = width / 2;
    std::vector<Point2D> vertices = {
        Point2D(-half_length, -half_width),
        Point2D(half_length, -half_width),
        Point2D(half_length, half_width),
        Point2D(-half_length, half_width)
    };

    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);

    std::vector<Point2D> rotated_vertices;
    for (const auto& vertex : vertices) {
        Point2D rotated_vertex;
        rotated_vertex.x = cos_angle * vertex.x - sin_angle * vertex.y;
        rotated_vertex.y = sin_angle * vertex.x + cos_angle * vertex.y;
        rotated_vertex.x += center.x;
        rotated_vertex.y += center.y;
        rotated_vertices.push_back(rotated_vertex);
    }

    return rotated_vertices;
}

bool point_in_polygon(double x, double y, const std::vector<std::pair<double, double>>& polygon) {
    bool inside = false;
    size_t j = polygon.size() - 1;
    for (size_t i = 0; i < polygon.size(); ++i) {
        double xi = polygon[i].first;
        double yi = polygon[i].second;
        double xj = polygon[j].first;
        double yj = polygon[j].second;

        bool intersect = ((yi > y) != (yj > y)) &&
                         (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (intersect) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

std::vector<std::pair<double, double>> draw_solid_polygon_on_grid(const std::vector<std::pair<double, double>>& polygon) {
    std::vector<std::pair<double, double>> occupied_cells;
    for (double i = left_bound + (cell_size / 2); i < right_bound - (cell_size / 2); i = i + cell_size) {
        for (double j = up_bound + (cell_size / 2); j < down_bound - (cell_size / 2); j = j + cell_size) {
            if (point_in_polygon(j, i, polygon)) {
                occupied_cells.emplace_back(i, j);
            }
        }
    }
    return occupied_cells;
}

class GridFutureCellsPublisherNode : public rclcpp::Node
{
public:
    GridFutureCellsPublisherNode() : Node("grid_future_cells_publisher_node"){
        this->declare_parameter("index", 0);
        this->get_parameter("index",index);
        // this->declare_parameter("publish_index", 0);
        // this->get_parameter("publish_index",publish_index);
        publisher_grid = this->create_publisher<nav_msgs::msg::GridCells>("/grid_future_cells_"+std::to_string(index), 10);
        publisher_grid_with_id=this->create_publisher<viplanning_msgs::msg::GridCellsWithId>("/grid_future_cells_"+std::to_string(index)+"_with_id",10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(update_rate), std::bind(&GridFutureCellsPublisherNode::timer_callback, this));
        subscription_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
            "/objects", 10, std::bind(&GridFutureCellsPublisherNode::predicted_objects_callback, this, std::placeholders::_1));
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

    

private:
    void timer_callback() {
        objects_cells.clear();
        objects_id.clear();
        if(objects.empty()) return;
        auto start_time=this->get_clock()->now();
        for (auto object : objects) {
            std::vector<std::pair<double, double>> polygon;
            for (auto point : object.first) polygon.push_back({point.x, point.y});
            auto object_occupancy=draw_solid_polygon_on_grid(polygon);
            for(int i=0;i<object_occupancy.size();i++) objects_id.push_back(object.second);
            objects_cells.push_back(object_occupancy);
        }
        auto end_time=this->get_clock()->now();
        runtime.push_back((end_time.seconds()-start_time.seconds())*1000.0); 
        auto msg = std::make_unique<nav_msgs::msg::GridCells>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "map";
        msg->cell_width = cell_size;
        msg->cell_height = cell_size;
        for (auto object_cell : objects_cells) {
            for (const auto& cell : object_cell) {
                geometry_msgs::msg::Point point;
                point.x = static_cast<double>(cell.second);
                point.y = static_cast<double>(cell.first);
                point.z = 0.0;
                msg->cells.push_back(point);
            }
        }
        publisher_grid->publish(*msg);
        viplanning_msgs::msg::GridCellsWithId grid_cells_with_id;
        grid_cells_with_id.grid_cells=*msg;
        grid_cells_with_id.objects_id.data=objects_id;
        publisher_grid_with_id->publish(grid_cells_with_id);
        RCLCPP_INFO(this->get_logger(), "Publishing GridCells message");
        //objects.clear();
    }

    void predicted_objects_callback(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg){
        objects.clear();
        for (int8_t i=0;i<msg->objects.size();i++) {
            auto object=msg->objects[i];
            if (object.kinematics.predicted_paths.size() <= 0) return;
            decltype(object.kinematics.predicted_paths[0]) max_confidence_path = object.kinematics.predicted_paths[0];
            for (auto path : object.kinematics.predicted_paths) if (path.confidence >= max_confidence_path.confidence) max_confidence_path = path;
            RCLCPP_INFO(this->get_logger(), "path size %d ",max_confidence_path.path.size());
            Quaternion p(max_confidence_path.path[index].orientation.w, max_confidence_path.path[index].orientation.x, max_confidence_path.path[index].orientation.y,
                    max_confidence_path.path[index].orientation.z);
            auto path_angles = quaternionToEuler(p);
            Point2D path_point = Point2D(max_confidence_path.path[index].position.x, max_confidence_path.path[index].position.y);
            auto path_res = get_2d_bbox_vertices(path_point, object.shape.dimensions.x, object.shape.dimensions.y, path_angles.yaw);
            objects.push_back({path_res,i});
        }
    }

    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr publisher_grid;
    rclcpp::Publisher<viplanning_msgs::msg::GridCellsWithId>::SharedPtr publisher_grid_with_id;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
    std::vector<std::vector<std::pair<double, double>>> objects_cells;
    std::vector<std::pair<std::vector<Point2D>,int8_t>> objects;

    std::vector<int8_t>objects_id;

    int index=0;

    int publish_index=0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridFutureCellsPublisherNode>();
    rclcpp::spin(node);
    //node->writeVectorToCSV(node->runtime,"/home/wodone/viplanning_ws/test_data/runtimr_data/high_generate.csv");
    rclcpp::shutdown();
    return 0;
}
