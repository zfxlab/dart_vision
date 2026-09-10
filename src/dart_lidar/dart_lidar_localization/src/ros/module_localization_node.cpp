#include "dart_lidar_localization/ros/module_localization_node.hpp"

#include <cmath>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <tf2_eigen/tf2_eigen.hpp>

#include "dart_lidar_localization/io/model_loader.hpp"

namespace dart_vision::lidar::localization {

ModuleLocalizationNode::ModuleLocalizationNode(const rclcpp::NodeOptions& options)
    : Node("module_localization", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    const auto path = declare_parameter<std::string>("model_path", "");
    const bool debug = declare_parameter<bool>("publish_candidate_model", true);
    model_ = ModelLoader::load(path);
    // 固定算法默认值集中定义在ModuleLocalizationParameters；不依赖基地配准门槛。
    localizer_ = std::make_unique<RailModuleLocalizer>(ModuleLocalizationParameters{}, model_);
    observation_publisher_ = create_publisher<dart_interfaces::msg::LidarObservation>(
        "lidar/observation", rclcpp::QoS(10).reliable());
    if (debug) {
        candidate_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "lidar/module/candidate_model", rclcpp::QoS(1).best_effort());
    }
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/accumulated",
        rclcpp::QoS(1).best_effort(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { cloudCallback(message); });
    RCLCPP_INFO(get_logger(),
                "Fixed-site module localization, model=%s (%zu points)",
                path.c_str(),
                model_->size());
}

void ModuleLocalizationNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
    dart_interfaces::msg::LidarObservation output;
    output.header.stamp = message->header.stamp;
    output.header.frame_id = rail_frame_;
    output.available = false;
    output.position_m = std::numeric_limits<double>::quiet_NaN();
    ModuleLocalizationResult result;
    try {
        const auto stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
        if (stamp_ns <= 0 || message->header.frame_id != reference_frame_) {
            throw std::runtime_error("expected positive timestamp in " + reference_frame_);
        }
        if (stamp_ns == last_stamp_ns_) {
            return; // 不将重复输入视为新的观测。
        }
        if (stamp_ns < last_stamp_ns_) {
            reference_from_rail_.reset(); // bag重新播放后重新获取静态关系。
        }
        last_stamp_ns_ = stamp_ns;
        if (!reference_from_rail_) {
            const auto transform =
                tf_buffer_.lookupTransform(reference_frame_, rail_frame_, tf2::TimePointZero);
            reference_from_rail_ = tf2::transformToEigen(transform.transform);
            RCLCPP_INFO(get_logger(),
                        "Cached fixed description transform %s <- %s",
                        reference_frame_.c_str(),
                        rail_frame_.c_str());
        }
        PointCloud::Ptr observation(new PointCloud);
        pcl::fromROSMsg(*message, *observation);
        result = localizer_->locate(observation, *reference_from_rail_);
        output.available = result.available;
        if (output.available) {
            output.position_m = result.position_m;
        }
        if (candidate_publisher_ && result.has_candidate) {
            Eigen::Isometry3d reference_from_module = *reference_from_rail_;
            reference_from_module.translate(Eigen::Vector3d(result.position_m, 0.0, 0.0));
            PointCloud placed;
            pcl::transformPointCloud(*model_, placed, reference_from_module.matrix().cast<float>());
            sensor_msgs::msg::PointCloud2 candidate;
            pcl::toROSMsg(placed, candidate);
            candidate.header = message->header;
            candidate_publisher_->publish(candidate);
        }
    } catch (const std::exception& error) {
        result.message = error.what();
    }
    observation_publisher_->publish(output);
    if (!output.available) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Module unavailable: %s", result.message.c_str());
    } else {
        RCLCPP_DEBUG(get_logger(),
                     "Module q=%.4fm rmse=%.4f overlap=%.3f",
                     result.position_m,
                     result.metrics.rmse_m,
                     result.metrics.overlap_ratio);
    }
}
} // namespace dart_vision::lidar::localization
