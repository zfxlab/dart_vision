#pragma once
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "dart_lidar_localization/calibration/base_registrar.hpp"
#include "dart_lidar_localization/calibration/confirmation_gate.hpp"
#include "dart_lidar_localization/calibration/result_writer.hpp"

namespace dart_vision::lidar::localization {
class BaseCalibrationNode : public rclcpp::Node {
public:
    explicit BaseCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
    std::string exportRecord(const LocalizationRecord& record);
    std::string model_path_, output_directory_, bag_name_;
    bool save_once_{true}, saved_{false};
    std::int64_t last_stamp_ns_{0};
    std::uint64_t measurement_id_{0};
    BaseRegistrationParameters parameters_;
    std::unique_ptr<BaseRegistrar> registrar_;
    std::optional<Eigen::Isometry3d> reference_from_base_;
    std::optional<LocalizationRecord> latest_candidate_;
    ConfirmationGate confirmation_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr candidate_publisher_,
        aligned_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_service_;
};
} // namespace dart_vision::lidar::localization
