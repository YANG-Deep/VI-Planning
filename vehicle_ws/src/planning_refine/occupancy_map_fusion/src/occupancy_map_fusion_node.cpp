#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include<eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <limits>

const int SIZE = 200;
const double CX = 99.5;
const double CY = 99.5;

Eigen::MatrixXi generate_occupancy_map(const std::vector<std::tuple<int, int, int, int>>& vehicles) {
    Eigen::MatrixXi grid = Eigen::MatrixXi::Zero(SIZE, SIZE);
    for (const auto& [x, y, w, h] : vehicles) {
        int x_end = std::min(x + w, SIZE);
        int y_end = std::min(y + h, SIZE);
        for (int i = y; i < y_end; ++i) {
            for (int j = x; j < x_end; ++j) {
                grid(i, j) = 1;
            }
        }
    }
    return grid;
}

Eigen::MatrixXi apply_transform(const Eigen::MatrixXi& points, double theta_deg, int dx, int dy) {
    double theta = theta_deg * M_PI / 180.0;
    Eigen::MatrixXd rotated(points.rows(), 2);
    for (int i = 0; i < points.rows(); ++i) {
        double rx = points(i, 0) - CX;
        double ry = points(i, 1) - CY;
        double x_rot = std::round(rx * std::cos(theta) - ry * std::sin(theta) + CX) + dx;
        double y_rot = std::round(rx * std::sin(theta) + ry * std::cos(theta) + CY) + dy;
        rotated(i, 0) = x_rot;
        rotated(i, 1) = y_rot;
    }
    return rotated.cast<int>();
}

std::tuple<double, std::tuple<double, int, int>, int> baseline_method(const Eigen::MatrixXi& road_points, const Eigen::Matrix<bool, SIZE, SIZE>& vehicle_mask) {
    auto start = std::chrono::high_resolution_clock::now();
    int best_score = -1;
    double best_theta = 0;
    int best_dx = 0;
    int best_dy = 0;

    std::vector<double> angles;
    for (int a = 0; a < 6; ++a) {
        if (a == 0) {
            angles.push_back(a);
        } else {
            angles.push_back(a);
            angles.push_back(-a);
        }
    }

    for (double theta_deg : angles) {
        Eigen::MatrixXi rotated = apply_transform(road_points, theta_deg, 0, 0);
        for (int dx = -5; dx <= 5; ++dx) {
            for (int dy = -5; dy <= 5; ++dy) {
                Eigen::MatrixXi translated = rotated.array() + Eigen::Array2i(dx, dy);
                int score = 0;
                for (int i = 0; i < translated.rows(); ++i) {
                    int x = translated(i, 0);
                    int y = translated(i, 1);
                    if (x >= 0 && x < SIZE && y >= 0 && y < SIZE && vehicle_mask(y, x)) {
                        score++;
                    }
                }
                if (score > best_score) {
                    best_score = score;
                    best_theta = theta_deg;
                    best_dx = dx;
                    best_dy = dy;
                }
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    return {runtime, {best_theta, best_dx, best_dy}, best_score};
}

std::tuple<double, std::tuple<double, int, int>, double> probabilistic_method(const Eigen::MatrixXi& road_points, const Eigen::Matrix<bool, SIZE, SIZE>& vehicle_mask) {
    Eigen::MatrixXd vehicle_prob = vehicle_mask.cast<double>() * 0.8 + Eigen::MatrixXd::Constant(SIZE, SIZE, 0.1);
    auto start = std::chrono::high_resolution_clock::now();
    double best_score = -std::numeric_limits<double>::infinity();
    double best_theta = 0;
    int best_dx = 0;
    int best_dy = 0;

    std::vector<double> angles;
    for (int a = 0; a < 6; ++a) {
        if (a == 0) {
            angles.push_back(a);
        } else {
            angles.push_back(a);
            angles.push_back(-a);
        }
    }

    for (double theta_deg : angles) {
        Eigen::MatrixXi rotated = apply_transform(road_points, theta_deg, 0, 0);
        for (int dx = -5; dx <= 5; ++dx) {
            for (int dy = -5; dy <= 5; ++dy) {
                Eigen::MatrixXi translated = rotated.array() + Eigen::Array2i(dx, dy);
                double score = 0;
                for (int i = 0; i < translated.rows(); ++i) {
                    int x = translated(i, 0);
                    int y = translated(i, 1);
                    if (x >= 0 && x < SIZE && y >= 0 && y < SIZE) {
                        score += std::log(vehicle_prob(y, x));
                    }
                }
                if (score > best_score) {
                    best_score = score;
                    best_theta = theta_deg;
                    best_dx = dx;
                    best_dy = dy;
                }
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    return {runtime, {best_theta, best_dx, best_dy}, best_score};
}

std::tuple<double, std::tuple<double, int, int>, int> optimization_method(const Eigen::MatrixXi& road_points, const Eigen::Matrix<bool, SIZE, SIZE>& vehicle_mask) {
    auto start = std::chrono::high_resolution_clock::now();
    int best_score = -1;
    double best_theta = 0;
    int best_dx = 0;
    int best_dy = 0;

    std::vector<double> angles;
    for (int a = 0; a < 6; ++a) {
        if (a == 0) {
            angles.push_back(a);
        } else {
            angles.push_back(a);
            angles.push_back(-a);
        }
    }

    for (double theta_deg : angles) {
        Eigen::MatrixXi rotated = apply_transform(road_points, theta_deg, 0, 0);
        for (int dx = -5; dx <= 5; ++dx) {
            for (int dy = -5; dy <= 5; ++dy) {
                Eigen::MatrixXi translated = rotated.array() + Eigen::Array2i(dx, dy);
                int score = 0;
                for (int i = 0; i < translated.rows(); ++i) {
                    int x = translated(i, 0);
                    int y = translated(i, 1);
                    if (x >= 0 && x < SIZE && y >= 0 && y < SIZE && vehicle_mask(y, x)) {
                        score++;
                    }
                }
                if (score > best_score) {
                    best_score = score;
                    best_theta = theta_deg;
                    best_dx = dx;
                    best_dy = dy;
                }
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    return {runtime, {best_theta, best_dx, best_dy}, best_score};
}

Eigen::MatrixXi create_transformed_map(const Eigen::MatrixXi& road_points, double theta_deg, int dx, int dy) {
    Eigen::MatrixXi transformed_map = Eigen::MatrixXi::Zero(SIZE, SIZE);
    Eigen::MatrixXi transformed = apply_transform(road_points, theta_deg, dx, dy);
    for (int i = 0; i < transformed.rows(); ++i) {
        int x = transformed(i, 0);
        int y = transformed(i, 1);
        if (x >= 0 && x < SIZE && y >= 0 && y < SIZE) {
            transformed_map(y, x) = 1;
        }
    }
    return transformed_map;
}

class OccupancyMapFusionNode : public rclcpp::Node {
public:
    OccupancyMapFusionNode() : Node("occupancy_map_fusion_node") {
        vehicle_map_sub_ = this->create_subscription<nav_msgs::msg::GridCells>(
            "/grid_future_cells_0", 10, std::bind(&OccupancyMapFusionNode::vehicle_map_callback, this, std::placeholders::_1));
        road_map_sub_ = this->create_subscription<nav_msgs::msg::GridCells>(
            "/grid_cells_with_noise_0", 10, std::bind(&OccupancyMapFusionNode::road_map_callback, this, std::placeholders::_1));
        fused_map_pub_ = this->create_publisher<nav_msgs::msg::GridCells>("fused_map", 10);
    }

private:
    void vehicle_map_callback(const nav_msgs::msg::GridCells::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),"receive vehicle map!");
        vehicle_map_ = Eigen::Matrix<bool, SIZE, SIZE>::Zero();
        for (const auto& cell : msg->cells) {
            int x = static_cast<int>(cell.x+CX);
            int y = static_cast<int>(cell.y+CY);
            if (x >= 0 && x < SIZE && y >= 0 && y < SIZE) {
                vehicle_map_(y, x) = true;
            }
        }
        process_maps();
    }

    void road_map_callback(const nav_msgs::msg::GridCells::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),"receive road map!");
        std::vector<std::tuple<int, int>> occupied_points;
        for (const auto& cell : msg->cells) {
            int x = static_cast<int>(cell.x+CX);
            int y = static_cast<int>(cell.y+CY);
            if (x >= 0 && x < SIZE && y >= 0 && y < SIZE) {
                occupied_points.emplace_back(x, y);
            }
        }
        road_points_ = Eigen::MatrixXi(occupied_points.size(), 2);
        for (size_t i = 0; i < occupied_points.size(); ++i) {
            road_points_(i, 0) = std::get<0>(occupied_points[i]);
            road_points_(i, 1) = std::get<1>(occupied_points[i]);
        }
        process_maps();
    }

    void process_maps() {
        if (!vehicle_map_.isZero() && !road_points_.isZero()) {
            auto [baseline_runtime, baseline_params, baseline_score] = baseline_method(road_points_, vehicle_map_);
            auto [prob_runtime, prob_params, prob_score] = probabilistic_method(road_points_, vehicle_map_);
            auto [opt_runtime, opt_params, opt_score] = optimization_method(road_points_, vehicle_map_);

            auto [theta, dx, dy] = opt_params;
            Eigen::MatrixXi transformed_map = create_transformed_map(road_points_, theta, dx, dy);

            nav_msgs::msg::GridCells fused_map_msg;
            fused_map_msg.header.stamp = this->now();
            fused_map_msg.header.frame_id = "map";
            fused_map_msg.cell_width = 1.0;
            fused_map_msg.cell_height = 1.0;

            for (int i = 0; i < SIZE; ++i) {
                for (int j = 0; j < SIZE; ++j) {
                    if (transformed_map(i, j) == 1) {
                        geometry_msgs::msg::Point cell;
                        cell.x = static_cast<double>(j);
                        cell.y = static_cast<double>(i);
                        cell.z = 0.0;
                        fused_map_msg.cells.push_back(cell);
                    }
                }
            }
            fused_map_pub_->publish(fused_map_msg);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr vehicle_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr road_map_sub_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr fused_map_pub_;

    Eigen::Matrix<bool, SIZE, SIZE> vehicle_map_;
    Eigen::MatrixXi road_points_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyMapFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}