#ifndef DART_LIDAR_PREPROCESSING_LIVOX_PREPROCESSOR_NODE_HPP
#define DART_LIDAR_PREPROCESSING_LIVOX_PREPROCESSOR_NODE_HPP

#include <chrono>
#include <deque>
#include <livox_interfaces/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace dart_vision::lidar {

class LivoxPreprocessorNode : public rclcpp::Node {
public:
    explicit LivoxPreprocessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    struct PendingCloud {
        livox_interfaces::msg::CustomMsg::ConstSharedPtr message;
        rclcpp::Time received_ros_time;
        std::chrono::steady_clock::time_point received_steady_time;
    };

    void cloudCallback(const livox_interfaces::msg::CustomMsg::ConstSharedPtr& message);
    void processPendingClouds();
    [[nodiscard]] bool preprocessAndPublish(const PendingCloud& pending);

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    double min_distance_m_{20.0};
    double max_distance_m_{30.0};
    bool voxel_grid_enabled_{true};
    double voxel_leaf_size_m_{0.01};
    double yaw_query_offset_s_{0.0};
    double time_bin_s_{0.002};
    double tf_wait_timeout_s_{0.15};
    std::size_t max_pending_clouds_{8U};

    std::deque<PendingCloud> pending_clouds_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    rclcpp::TimerBase::SharedPtr pending_timer_;
};

} // namespace dart_vision::lidar

#endif // DART_LIDAR_PREPROCESSING_LIVOX_PREPROCESSOR_NODE_HPP
