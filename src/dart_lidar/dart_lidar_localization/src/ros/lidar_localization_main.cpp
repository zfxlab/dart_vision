#include "dart_lidar_localization/ros/lidar_localization_node.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<dart_vision::lidar::localization::LidarLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
