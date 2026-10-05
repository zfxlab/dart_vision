#include "dart_stereo/stereo_triangulator_node.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <utility>

namespace dart_vision::stereo {
namespace {
double stampSeconds(const std_msgs::msg::Header& header) {
    return rclcpp::Time(header.stamp).seconds();
}

struct NormalizedObservation {
    double x{};
    double y{};
};

std::optional<NormalizedObservation>
normalizeObservation(const dart_interfaces::msg::GreenLightDetection& detection,
                     const sensor_msgs::msg::CameraInfo& info,
                     const std::string& expected_frame) {
    if (detection.header.frame_id != expected_frame || info.header.frame_id != expected_frame ||
        info.width == 0U || info.height == 0U || !std::isfinite(detection.center_x_px) ||
        !std::isfinite(detection.center_y_px) || !std::isfinite(detection.radius_px) ||
        detection.center_x_px < 0.0F || detection.center_y_px < 0.0F ||
        detection.center_x_px >= static_cast<float>(info.width) ||
        detection.center_y_px >= static_cast<float>(info.height) || detection.radius_px <= 0.0F) {
        return std::nullopt;
    }

    const double scale = info.k[8];
    if (!std::isfinite(scale) || scale == 0.0) {
        return std::nullopt;
    }
    const double fx = info.k[0] / scale;
    const double fy = info.k[4] / scale;
    const double cx = info.k[2] / scale;
    const double cy = info.k[5] / scale;
    if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy) ||
        fx <= 0.0 || fy <= 0.0) {
        return std::nullopt;
    }

    return NormalizedObservation{(static_cast<double>(detection.center_x_px) - cx) / fx,
                                 (static_cast<double>(detection.center_y_px) - cy) / fy};
}
} // namespace

StereoTriangulatorNode::StereoTriangulatorNode(const rclcpp::NodeOptions& options)
    : Node("stereo_triangulator", options) {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "Loaded at startup; restart the node to change this parameter";

    const std::string left_topic =
        declare_parameter<std::string>("left_detection_topic", "/left_camera/detection", read_only);
    const std::string right_topic = declare_parameter<std::string>(
        "right_detection_topic", "/right_camera/detection", read_only);
    const std::string left_camera_info_topic = declare_parameter<std::string>(
        "left_camera_info_topic", "/left_camera/camera_info", read_only);
    const std::string right_camera_info_topic = declare_parameter<std::string>(
        "right_camera_info_topic", "/right_camera/camera_info", read_only);
    const std::string result_topic =
        declare_parameter<std::string>("result_topic", "stereo_target", read_only);
    left_frame_id_ =
        declare_parameter<std::string>("left_frame_id", "left_camera_optical_frame", read_only);
    right_frame_id_ =
        declare_parameter<std::string>("right_frame_id", "right_camera_optical_frame", read_only);
    reference_frame_ =
        declare_parameter<std::string>("reference_frame", "stereo_camera_center_link", read_only);
    max_pair_delta_s_ = declare_parameter<double>("max_pair_delta_s", 0.03, read_only);
    const std::int64_t configured_queue_size = declare_parameter<int>("queue_size", 10, read_only);

    StereoTriangulatorConfig config;
    config.baseline_m = declare_parameter<double>("baseline_m", config.baseline_m, read_only);
    config.min_normalized_disparity = declare_parameter<double>(
        "min_normalized_disparity", config.min_normalized_disparity, read_only);
    config.min_distance_m =
        declare_parameter<double>("min_distance_m", config.min_distance_m, read_only);
    config.max_distance_m =
        declare_parameter<double>("max_distance_m", config.max_distance_m, read_only);
    config.max_vertical_residual =
        declare_parameter<double>("max_vertical_residual", config.max_vertical_residual, read_only);

    if (left_topic.empty() || right_topic.empty() || left_camera_info_topic.empty() ||
        right_camera_info_topic.empty() || result_topic.empty() || left_frame_id_.empty() ||
        right_frame_id_.empty() || reference_frame_.empty() || !std::isfinite(max_pair_delta_s_) ||
        max_pair_delta_s_ < 0.0 || configured_queue_size <= 0 ||
        left_frame_id_ == right_frame_id_) {
        throw std::invalid_argument("Invalid stereo triangulator node configuration");
    }
    queue_size_ = static_cast<std::size_t>(configured_queue_size);
    triangulator_ = std::make_unique<StereoTriangulator>(config);

    result_publisher_ = create_publisher<StereoTarget>(result_topic, rclcpp::SensorDataQoS());
    left_subscription_ = create_subscription<GreenLightDetection>(
        left_topic, rclcpp::SensorDataQoS(), [this](GreenLightDetection::ConstSharedPtr message) {
            observationCallback(std::move(message), true);
        });
    right_subscription_ = create_subscription<GreenLightDetection>(
        right_topic, rclcpp::SensorDataQoS(), [this](GreenLightDetection::ConstSharedPtr message) {
            observationCallback(std::move(message), false);
        });
    left_camera_info_subscription_ =
        create_subscription<CameraInfo>(left_camera_info_topic,
                                        rclcpp::SensorDataQoS(),
                                        [this](CameraInfo::ConstSharedPtr message) {
                                            cameraInfoCallback(std::move(message), true);
                                        });
    right_camera_info_subscription_ =
        create_subscription<CameraInfo>(right_camera_info_topic,
                                        rclcpp::SensorDataQoS(),
                                        [this](CameraInfo::ConstSharedPtr message) {
                                            cameraInfoCallback(std::move(message), false);
                                        });

    RCLCPP_INFO(get_logger(),
                "Stereo disparity: left='%s', right='%s', left_info='%s', right_info='%s', "
                "output='%s'",
                left_topic.c_str(),
                right_topic.c_str(),
                left_camera_info_topic.c_str(),
                right_camera_info_topic.c_str(),
                result_topic.c_str());
}

void StereoTriangulatorNode::observationCallback(const GreenLightDetection::ConstSharedPtr& message,
                                                 const bool is_left) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    auto& queue = is_left ? left_queue_ : right_queue_;
    queue.push_back(message);
    while (queue.size() > queue_size_) {
        queue.pop_front();
    }
    matchQueuedObservations();
}

void StereoTriangulatorNode::cameraInfoCallback(CameraInfo::ConstSharedPtr message,
                                                const bool is_left) {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    (is_left ? left_camera_info_ : right_camera_info_) = std::move(message);
}

void StereoTriangulatorNode::matchQueuedObservations() {
    while (!left_queue_.empty() && !right_queue_.empty()) {
        std::size_t best_left = 0U;
        std::size_t best_right = 0U;
        double best_delta = std::numeric_limits<double>::infinity();
        for (std::size_t left_index = 0; left_index < left_queue_.size(); ++left_index) {
            for (std::size_t right_index = 0; right_index < right_queue_.size(); ++right_index) {
                const double delta = std::abs(stampSeconds(left_queue_[left_index]->header) -
                                              stampSeconds(right_queue_[right_index]->header));
                if (delta < best_delta) {
                    best_delta = delta;
                    best_left = left_index;
                    best_right = right_index;
                }
            }
        }

        if (best_delta <= max_pair_delta_s_) {
            const auto left = left_queue_[best_left];
            const auto right = right_queue_[best_right];
            left_queue_.erase(left_queue_.begin(), left_queue_.begin() + best_left + 1U);
            right_queue_.erase(right_queue_.begin(), right_queue_.begin() + best_right + 1U);
            processPair(*left, *right);
            continue;
        }

        const double left_stamp = stampSeconds(left_queue_.front()->header);
        const double right_stamp = stampSeconds(right_queue_.front()->header);
        if (left_stamp + max_pair_delta_s_ < right_stamp) {
            left_queue_.pop_front();
        } else if (right_stamp + max_pair_delta_s_ < left_stamp) {
            right_queue_.pop_front();
        } else {
            break;
        }
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Dropping unmatched stereo observation; closest timestamp delta %.4f s",
            best_delta);
    }
}

void StereoTriangulatorNode::processPair(const GreenLightDetection& left,
                                         const GreenLightDetection& right) {
    if (left.header.frame_id != left_frame_id_ || right.header.frame_id != right_frame_id_) {
        publishFailure(left, right, StereoTarget::INVALID);
        return;
    }
    if (left.status == GreenLightDetection::CLOSED && right.status == GreenLightDetection::CLOSED) {
        publishFailure(left, right, StereoTarget::CLOSED);
        return;
    }
    if (left.status != GreenLightDetection::DETECTED ||
        right.status != GreenLightDetection::DETECTED) {
        publishFailure(left, right, StereoTarget::INVALID);
        return;
    }

    CameraInfo::ConstSharedPtr left_info;
    CameraInfo::ConstSharedPtr right_info;
    {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        left_info = left_camera_info_;
        right_info = right_camera_info_;
    }
    if (!left_info || !right_info) {
        publishFailure(left, right, StereoTarget::INVALID);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Stereo CameraInfo has not been received");
        return;
    }

    const auto left_normalized = normalizeObservation(left, *left_info, left_frame_id_);
    const auto right_normalized = normalizeObservation(right, *right_info, right_frame_id_);
    if (!left_normalized || !right_normalized) {
        publishFailure(left, right, StereoTarget::INVALID);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Invalid stereo pixel observation or CameraInfo");
        return;
    }

    StereoTriangulationDiagnostics diagnostics;
    const auto result = triangulator_->triangulate(left_normalized->x,
                                                   left_normalized->y,
                                                   right_normalized->x,
                                                   right_normalized->y,
                                                   &diagnostics);
    if (!result) {
        publishFailure(left, right, StereoTarget::INVALID);
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            200,
            "Stereo disparity rejected: reason=%s, pair_delta=%.6f s, pixel_disparity=%.6f px, "
            "normalized_disparity=%.9f, vertical_residual=%.9f, forward=%.6f m, "
            "horizontal=%.6f m, distance=%.6f m, yaw=%.6f rad",
            stereoTriangulationRejectionName(diagnostics.rejection),
            std::abs(stampSeconds(left.header) - stampSeconds(right.header)),
            static_cast<double>(left.center_x_px) - static_cast<double>(right.center_x_px),
            diagnostics.normalized_disparity,
            diagnostics.vertical_residual,
            diagnostics.forward_m,
            diagnostics.horizontal_right_m,
            diagnostics.horizontal_distance_m,
            diagnostics.yaw_rad);
        return;
    }

    const auto left_stamp = rclcpp::Time(left.header.stamp).nanoseconds();
    const auto right_stamp = rclcpp::Time(right.header.stamp).nanoseconds();
    StereoTarget message;
    message.header = left.header;
    message.header.stamp = rclcpp::Time(left_stamp + (right_stamp - left_stamp) / 2);
    message.header.frame_id = reference_frame_;
    message.status = StereoTarget::VALID;
    message.position.x = result->forward_m;
    message.position.y = -result->horizontal_right_m;
    message.position.z = 0.0;
    message.distance = result->horizontal_distance_m;
    message.yaw = result->yaw_rad;
    result_publisher_->publish(message);
}

void StereoTriangulatorNode::publishFailure(const GreenLightDetection& left,
                                            const GreenLightDetection& right,
                                            const std::uint8_t status) {
    StereoTarget message;
    message.header = left.header;
    const auto left_stamp = rclcpp::Time(left.header.stamp).nanoseconds();
    const auto right_stamp = rclcpp::Time(right.header.stamp).nanoseconds();
    message.header.stamp = rclcpp::Time(left_stamp + (right_stamp - left_stamp) / 2);
    message.header.frame_id = reference_frame_;
    message.status = status;
    result_publisher_->publish(message);
}

} // namespace dart_vision::stereo
