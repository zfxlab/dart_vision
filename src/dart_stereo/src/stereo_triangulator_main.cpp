#include "dart_stereo/stereo_triangulator_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<dart_vision::stereo::StereoTriangulatorNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
