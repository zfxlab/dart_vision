#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dart_vision_lidar_localization/ros/mid70_localization_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dart_vision::lidar::Mid70LocalizationNode>(rclcpp::NodeOptions{});
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2U);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
