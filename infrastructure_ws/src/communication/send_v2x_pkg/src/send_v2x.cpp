#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

class SendV2X : public rclcpp::Node {
public:
    SendV2X() : Node("Send_V2X") {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            throw std::runtime_error("Failed to create socket");
        }

        memset(&dest_addr_, 0, sizeof(dest_addr_));
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(30300);
        if (inet_pton(AF_INET, "192.168.20.199", &dest_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid address");
            close(sock_);
            throw std::runtime_error("Invalid address");
        }

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/occupancy_map_bin", 10, std::bind(&SendV2X::listener_callback, this, std::placeholders::_1));
    }

    ~SendV2X() {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: %zu", msg->data.size());

        std::string hex_str;
        for (char c : msg->data) {
            std::stringstream ss;
            ss << std::hex << std::setw(2) << std::setfill('0') 
               << static_cast<int>(static_cast<unsigned char>(c));
            hex_str += ss.str();
        }

        std::vector<uint8_t> send_data;
        for (size_t i = 0; i < hex_str.size(); i += 2) {
            std::string byte_str = hex_str.substr(i, 2);
            try {
                auto byte = static_cast<uint8_t>(std::stoul(byte_str, nullptr, 16));
                send_data.push_back(byte);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Hex conversion error: %s", e.what());
                return;
            }
        }


        ssize_t sent = sendto(
            sock_, 
            send_data.data(), 
            send_data.size(),
            0,
            reinterpret_cast<struct sockaddr*>(&dest_addr_),
            sizeof(dest_addr_));

        if (sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "Send failed: %s", strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent %zd bytes", sent);
        }
    }

    int sock_;
    int count=0;
    struct sockaddr_in dest_addr_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<SendV2X>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Send_V2X"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
