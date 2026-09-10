#include "dart_lidar_localization/ros/base_calibration_node.hpp"
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dart_vision::lidar::localization::BaseCalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
