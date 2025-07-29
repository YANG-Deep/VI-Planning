#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>

// Function to convert a single hex byte to a character
char hexByteToChar(const std::string& hexByte) {
    unsigned int byte;
    std::stringstream ss;
    ss << std::hex << hexByte;
    ss >> byte;
    return static_cast<char>(byte);
}

// Function to convert a space-separated string of hex bytes to a decoded string
std::string hexToString(const std::string& hexInput) {
    std::stringstream ss(hexInput);
    std::string hexByte;
    std::string result;

    while (ss >> hexByte) {
        result += hexByteToChar(hexByte);
    }

    return result;
}

class HexDecodeNode : public rclcpp::Node {
public:
    HexDecodeNode() : Node("hex_decode_node") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/hex_string", 10, std::bind(&HexDecodeNode::hexStringCallback, this, std::placeholders::_1));
        publisher=this->create_publisher<std_msgs::msg::String>("/occupancy_map_bin",10);
    }

private:
    std::string addSpaceEveryTwoChars(const std::string& input) {
        std::string result;
        for (size_t i = 0; i < input.length(); ++i) {
            result += input[i];
            if ((i + 1) % 2 == 0 && i != input.length() - 1) {
                result += ' ';
            }
        }
        return result;
    }
  
    void hexStringCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string hexInput;
        RCLCPP_INFO(this->get_logger(), "Receive size %d ",msg->data.size());
        for(int i=8;i<msg->data.size();i++) hexInput.push_back(msg->data[i]);
        RCLCPP_INFO(this->get_logger(), "hexinput size %d ",hexInput.size());
        hexInput=addSpaceEveryTwoChars(hexInput);
        std::string decodedString = hexToString(hexInput);
        RCLCPP_INFO(this->get_logger(), "Receive  %s ",hexInput.c_str());
        RCLCPP_INFO(this->get_logger(), "Decoded String size: %d ",decodedString.size());
        std_msgs::msg::String msg_str;
        msg_str.data=decodedString;
        publisher->publish(msg_str);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexDecodeNode>());
    rclcpp::shutdown();
    return 0;
}
