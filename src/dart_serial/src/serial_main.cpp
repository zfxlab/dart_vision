#include "dart_serial/serial_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dart_vision::serial::SerialNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
