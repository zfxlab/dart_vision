#pragma once

#include <dart_interfaces/msg/lidar_observation.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "dart_lidar_localization/module/rail_module_localizer.hpp"

namespace dart_vision::lidar::localization {
class ModuleLocalizationNode : public rclcpp::Node {
public:
    explicit ModuleLocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
    std::string reference_frame_{"base_nominal_link"};
    std::string rail_frame_{"rail_origin_link"};
    PointCloud::Ptr model_;
    std::unique_ptr<RailModuleLocalizer> localizer_;
    std::optional<Eigen::Isometry3d> reference_from_rail_;
    std::int64_t last_stamp_ns_{0};
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<dart_interfaces::msg::LidarObservation>::SharedPtr observation_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr candidate_publisher_;
};
} // namespace dart_vision::lidar::localization
