#include "dart_aiming/aiming_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dart_vision::aiming::AimingNode>());
    rclcpp::shutdown();
    return 0;
}
