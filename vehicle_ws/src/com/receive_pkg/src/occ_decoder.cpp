#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libssh/libssh.h>
#include <chrono>
#include <string>
#include <regex>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

class OccDecoder : public rclcpp::Node {
public:
    OccDecoder() : Node("Occ_decoder") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("Get_V2X", 10);
        
        session = ssh_new();
        if (!session) {
            RCLCPP_ERROR(this->get_logger(), "Error creating SSH session");
            exit(-1);
        }

        ssh_options_set(session, SSH_OPTIONS_HOST, "192.168.1.199");
        ssh_options_set(session, SSH_OPTIONS_PORT, &port);
        ssh_options_set(session, SSH_OPTIONS_USER, "root");

        int rc = ssh_connect(session);
        if (rc != SSH_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error connecting: %s", ssh_get_error(session));
            ssh_free(session);
            exit(-1);
        }

        rc = ssh_userauth_password(session, nullptr, "root");
        if (rc != SSH_AUTH_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Authentication failed: %s", ssh_get_error(session));
            ssh_disconnect(session);
            ssh_free(session);
            exit(-1);
        }

        channel = ssh_channel_new(session);
        if (!channel) {
            RCLCPP_ERROR(this->get_logger(), "Channel creation failed");
            exit(-1);
        }

        rc = ssh_channel_open_session(channel);
        if (rc != SSH_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error opening channel");
            exit(-1);
        }

        ssh_channel_set_blocking(channel,1);

        rc = ssh_channel_request_pty(channel);       
         // if (rc != SSH_OK) {
        //     std::cerr << "Error requesting PTY: " << ssh_get_error(session) << std::endl;
        //     ssh_channel_close(channel);
        //     ssh_channel_free(channel);
        //     ssh_disconnect(session);
        //     ssh_free(session);
        //     return;
        // }

        ssh_channel_request_exec(channel,"ssty raw -echo");

        std::string command = "v2xdump -Q in --aid -1 -X";
        auto rc_=ssh_channel_request_exec(channel, command.c_str());
        if(rc_!=SSH_OK){
            RCLCPP_ERROR(this->get_logger(), "error! %s ",ssh_get_error(session));
        }

        timer_ = this->create_wall_timer(
            100ms, std::bind(&OccDecoder::timer_callback, this));

       
    }

    ~OccDecoder() {
        ssh_channel_close(channel);
        ssh_channel_free(channel);
        ssh_disconnect(session);
        ssh_free(session);
    }

private:
    void timer_callback() {
        char buffer[10240];
        int nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
        RCLCPP_INFO(this->get_logger(), "nbytes: %d",nbytes);
        if (nbytes > 0) {
            RCLCPP_INFO(this->get_logger(), "get msg!");
            std::string raw_msg(buffer, nbytes);
            process_data(raw_msg);
        }
    }

    void process_data(const std::string& data) {
        size_t first_0000 = data.find("0000:");
        if (first_0000 == std::string::npos) return;
        
        size_t second_0000 = data.find("0000:", first_0000 + 5);
        if (second_0000 == std::string::npos) return;

        std::string message = data.substr(second_0000 + 6);
        
        message = std::regex_replace(message, std::regex("[0-9a-fA-F]{4}:"), "");
        message.erase(std::remove_if(message.begin(), message.end(), 
                     [](char c){ return c == ']' || c == '\'' || c == ' ' || c == '\n'; }), 
                     message.end());

        if (message.length() > 12) {
            std::string hex_str = message.substr(12);
            std::string decoded = hex_to_ascii(hex_str);
            
            auto msg = std_msgs::msg::String();
            msg.data = decoded;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing");
        }
    }

    std::string hex_to_ascii(const std::string& hex) {
        std::string result;
        for (size_t i = 0; i < hex.length(); i += 2) {
            std::string byteString = hex.substr(i, 2);
            char byte = static_cast<char>(strtol(byteString.c_str(), nullptr, 16));
            if (byte >= 32 && byte <= 126) {  
                result += byte;
            }
        }
        return result;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    ssh_session session;
    ssh_channel channel;
    int port = 22;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccDecoder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
