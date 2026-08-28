#include "dart_camera/green_light_detector_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<dart_vision::camera::GreenLightDetectorNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
