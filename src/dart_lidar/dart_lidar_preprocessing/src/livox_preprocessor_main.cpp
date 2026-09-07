#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "dart_lidar_preprocessing/livox_preprocessor_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dart_vision::lidar::LivoxPreprocessorNode>());
    rclcpp::shutdown();
    return 0;
}
