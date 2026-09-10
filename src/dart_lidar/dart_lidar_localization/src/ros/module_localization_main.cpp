#include "dart_lidar_localization/ros/module_localization_node.hpp"
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dart_vision::lidar::localization::ModuleLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
