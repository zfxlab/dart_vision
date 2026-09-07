#ifndef DART_LIDAR_PREPROCESSING_LIVOX_PREPROCESSOR_NODE_HPP
#define DART_LIDAR_PREPROCESSING_LIVOX_PREPROCESSOR_NODE_HPP

#include <livox_interfaces/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace dart_vision::lidar {

class LivoxPreprocessorNode : public rclcpp::Node {
public:
    explicit LivoxPreprocessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void cloudCallback(const livox_interfaces::msg::CustomMsg::ConstSharedPtr& message);

    std::string input_topic_;
    std::string output_topic_;
    double min_distance_m_{20.0};
    double max_distance_m_{30.0};
    bool voxel_grid_enabled_{true};
    double voxel_leaf_size_m_{0.01};

    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

} // namespace dart_vision::lidar

#endif // DART_LIDAR_PREPROCESSING_LIVOX_PREPROCESSOR_NODE_HPP
