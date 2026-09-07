#include "dart_lidar_accumulation/lidar_accumulator_node.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dart_vision::lidar::LidarAccumulatorNode>());
    rclcpp::shutdown();
    return 0;
}
