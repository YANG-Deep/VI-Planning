#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <cmath>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

double left_bound = -100;
double right_bound = 100;
double up_bound = -100;
double down_bound = 100;
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

class GridCellsPublisher : public rclcpp::Node {
public:
    GridCellsPublisher() : Node("grid_cells_publisher") {
        publisher_grid_car = this->create_publisher<nav_msgs::msg::GridCells>("grid_cells_car_topic", 10);
        publisher_grid_person = this->create_publisher<nav_msgs::msg::GridCells>("grid_cells_person_topic", 10);
        publisher_grid = this->create_publisher<nav_msgs::msg::GridCells>("/grid_cells_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(update_rate), std::bind(&GridCellsPublisher::timer_callback, this));
        subscription_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
            "/objects", 10, std::bind(&GridCellsPublisher::topic_callback, this, std::placeholders::_1));
    }

private:
    void timer_callback() {
        objects_cells_car.clear();
        objects_cells_person.clear();
        objects_cells.clear();
        for (auto object : objects_car) {
            std::vector<std::pair<double, double>> polygon;
            for (auto point : object) polygon.push_back({point.x, point.y});
            objects_cells_car.push_back(draw_solid_polygon_on_grid(polygon));
        }
        for (auto object : objects_person) {
            std::vector<std::pair<double, double>> polygon;
            for (auto point : object) polygon.push_back({point.x, point.y});
            objects_cells_person.push_back(draw_solid_polygon_on_grid(polygon));
        }
        for (auto object_cell : objects_cells_car) objects_cells.push_back(object_cell);
        for (auto object_cell : objects_cells_person) objects_cells.push_back(object_cell);
        auto msg_car = std::make_unique<nav_msgs::msg::GridCells>();
        auto msg_person = std::make_unique<nav_msgs::msg::GridCells>();
        auto msg_raw = std::make_unique<nav_msgs::msg::GridCells>();
        msg_car->header.stamp = this->get_clock()->now();
        msg_car->header.frame_id = "map";
        msg_car->cell_width = cell_size;
        msg_car->cell_height = cell_size;
        msg_person->header.stamp = this->get_clock()->now();
        msg_person->header.frame_id = "map";
        msg_person->cell_width = cell_size;
        msg_person->cell_height = cell_size;
        msg_raw->header.stamp = this->get_clock()->now();
        msg_raw->header.frame_id = "map";
        msg_raw->cell_width = cell_size;
        msg_raw->cell_height = cell_size;
        for (auto object_cell : objects_cells_car) {
            for (const auto& cell : object_cell) {
                geometry_msgs::msg::Point point;
                point.x = static_cast<double>(cell.second);
                point.y = static_cast<double>(cell.first);
                point.z = 0.0;
                msg_car->cells.push_back(point);
            }
        }
        for (auto object_cell : objects_cells_person) {
            for (const auto& cell : object_cell) {
                geometry_msgs::msg::Point point;
                point.x = static_cast<double>(cell.second);
                point.y = static_cast<double>(cell.first);
                point.z = 0.0;
                msg_person->cells.push_back(point);
            }
        }
        for (auto object_cell : objects_cells) {
            for (const auto& cell : object_cell) {
                geometry_msgs::msg::Point point;
                point.x = static_cast<double>(cell.second);
                point.y = static_cast<double>(cell.first);
                point.z = 0.0;
                msg_raw->cells.push_back(point);
            }
        }
        publisher_grid_car->publish(std::move(msg_car));
        publisher_grid_person->publish(std::move(msg_person));
        publisher_grid->publish(std::move(msg_raw));
        RCLCPP_INFO(this->get_logger(), "Publishing GridCells message");
        objects_car.clear();
        objects_person.clear();
    }

    void topic_callback(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
        for (auto object : msg->objects) {
            // current occupancy grid
            Quaternion q(object.kinematics.initial_pose_with_covariance.pose.orientation.w, object.kinematics.initial_pose_with_covariance.pose.orientation.x
                , object.kinematics.initial_pose_with_covariance.pose.orientation.y, object.kinematics.initial_pose_with_covariance.pose.orientation.z);
            EulerAngles angles = quaternionToEuler(q);
            Point2D center = Point2D(object.kinematics.initial_pose_with_covariance.pose.position.x, object.kinematics.initial_pose_with_covariance.pose.position.y);
            auto res = get_2d_bbox_vertices(center, object.shape.dimensions.x, object.shape.dimensions.y, angles.yaw);
            decltype(object.classification[0]) max_probability_classification = object.classification[0];
            for (int i = 1; i < object.classification.size(); i++) {
                if (object.classification[i].probability >= max_probability_classification.probability) max_probability_classification = object.classification[i];
            }
            if (max_probability_classification.label == max_probability_classification.PEDESTRIAN) objects_person.push_back(res);
            else objects_car.push_back(res);
        }
    }

    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr publisher_grid_car;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr publisher_grid_person;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr publisher_grid;
    std::unique_ptr<std::vector<rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr>> publisher_grid_future;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr subscription_;
    std::vector<std::vector<std::pair<double, double>>> objects_cells_car;
    std::vector<std::vector<std::pair<double, double>>> objects_cells_person;
    std::vector<std::vector<std::pair<double, double>>> objects_cells;
    std::vector<std::vector<Point2D>> objects_car;
    std::vector<std::vector<Point2D>> objects_person;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridCellsPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
