#include "dart_stereo/stereo_triangulator_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <limits>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

namespace dart_vision::stereo {
namespace {
double stampSeconds(const std_msgs::msg::Header& header) {
    return rclcpp::Time(header.stamp).seconds();
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
    const std::string result_topic =
        declare_parameter<std::string>("result_topic", "stereo_target", read_only);
    left_frame_id_ =
        declare_parameter<std::string>("left_frame_id", "left_camera_optical_frame", read_only);
    right_frame_id_ =
        declare_parameter<std::string>("right_frame_id", "right_camera_optical_frame", read_only);
    reference_frame_ =
        declare_parameter<std::string>("reference_frame", "stereo_camera_center_link", read_only);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    const auto left_info_topic = declare_parameter<std::string>(
        "left_camera_info_topic", "/left_camera/camera_info", read_only);
    const auto right_info_topic = declare_parameter<std::string>(
        "right_camera_info_topic", "/right_camera/camera_info", read_only);
    left_info_sub_ = create_subscription<Info>(
        left_info_topic, rclcpp::SensorDataQoS(), [this](Info::ConstSharedPtr msg) {
            left_infos_.push_back(msg);
            while (left_infos_.size() > 100)
                left_infos_.pop_front();
            matchQueuedObservations();
        });
    right_info_sub_ = create_subscription<Info>(
        right_info_topic, rclcpp::SensorDataQoS(), [this](Info::ConstSharedPtr msg) {
            right_infos_.push_back(msg);
            while (right_infos_.size() > 100)
                right_infos_.pop_front();
            matchQueuedObservations();
        });
    max_pair_delta_s_ = declare_parameter<double>("max_pair_delta_s", 0.03, read_only);
    const std::int64_t configured_queue_size = declare_parameter<int>("queue_size", 10, read_only);

    StereoTriangulatorConfig config;
    config.min_ray_angle_deg =
        declare_parameter<double>("min_ray_angle_deg", config.min_ray_angle_deg, read_only);
    config.min_depth_m = declare_parameter<double>("min_depth_m", config.min_depth_m, read_only);
    config.max_distance_m =
        declare_parameter<double>("max_distance_m", config.max_distance_m, read_only);
    config.max_ray_gap_m =
        declare_parameter<double>("max_ray_gap_m", config.max_ray_gap_m, read_only);

    if (left_topic.empty() || right_topic.empty() || result_topic.empty() ||
        left_frame_id_.empty() || right_frame_id_.empty() || reference_frame_.empty() ||
        !std::isfinite(max_pair_delta_s_) || max_pair_delta_s_ < 0.0 ||
        configured_queue_size <= 0 || left_frame_id_ == right_frame_id_) {
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

    RCLCPP_INFO(get_logger(),
                "Stereo triangulator: left='%s', right='%s', output='%s'",
                left_topic.c_str(),
                right_topic.c_str(),
                result_topic.c_str());
}

void StereoTriangulatorNode::observationCallback(const GreenLightDetection::ConstSharedPtr& message,
                                                 const bool is_left) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    auto& queue = is_left ? left_queue_ : right_queue_;
    queue.push_back(message);
    while (queue.size() > queue_size_)
        queue.pop_front();
    matchQueuedObservations();
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
            auto has_info = [](const auto& infos, const auto& detection) {
                return std::any_of(infos.begin(), infos.end(), [&](const auto& info) {
                    return info->header.stamp == detection->header.stamp;
                });
            };
            if (left->status == GreenLightDetection::DETECTED &&
                right->status == GreenLightDetection::DETECTED &&
                (!has_info(left_infos_, left) || !has_info(right_infos_, right))) {
                if (now().seconds() -
                        std::min(stampSeconds(left->header), stampSeconds(right->header)) <
                    0.2)
                    return;
            }
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
    if (left.status != GreenLightDetection::DETECTED) {
        publishFailure(left, right, StereoTarget::INVALID);
        return;
    }
    if (right.status != GreenLightDetection::DETECTED) {
        publishFailure(left, right, StereoTarget::INVALID);
        return;
    }

    const auto lb = bearing(left, left_infos_);
    const auto rb = bearing(right, right_infos_);
    if (!lb || !rb) {
        publishFailure(left, right, StereoTarget::INVALID);
        return;
    }
    const auto l = rclcpp::Time(left.header.stamp).nanoseconds();
    const auto r = rclcpp::Time(right.header.stamp).nanoseconds();
    const rclcpp::Time measurement_time(l + (r - l) / 2);
    cv::Vec3d left_origin, right_origin, left_direction, right_direction;
    try {
        // 光心使用 TF 的平移，视线是自由向量，仅使用 TF 的旋转。
        const auto transform_ray = [&](const std::string& frame,
                                       const cv::Vec3d& bearing,
                                       cv::Vec3d& origin,
                                       cv::Vec3d& direction) {
            const auto transform =
                tf_buffer_->lookupTransform(reference_frame_, frame, measurement_time);
            const auto& t = transform.transform.translation;
            origin = {t.x, t.y, t.z};
            geometry_msgs::msg::Vector3Stamped optical_direction, center_direction;
            optical_direction.vector.x = bearing[0];
            optical_direction.vector.y = bearing[1];
            optical_direction.vector.z = bearing[2];
            tf2::doTransform(optical_direction, center_direction, transform);
            const auto& v = center_direction.vector;
            direction = {v.x, v.y, v.z};
        };
        transform_ray(left_frame_id_, *lb, left_origin, left_direction);
        transform_ray(right_frame_id_, *rb, right_origin, right_direction);
    } catch (const tf2::TransformException& error) {
        publishFailure(left, right, StereoTarget::INVALID);
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             2000,
                             "Stereo reference transform unavailable: %s",
                             error.what());
        return;
    }
    const auto result =
        triangulator_->triangulate(left_origin, left_direction, right_origin, right_direction);
    if (!result) {
        publishFailure(left, right, StereoTarget::INVALID);
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             2000,
                             "Stereo triangulation rejected the paired viewing rays");
        return;
    }

    StereoTarget message;
    message.header = left.header;
    message.header.stamp = measurement_time;
    message.header.frame_id = reference_frame_;
    message.status = StereoTarget::VALID;
    message.position.x = result->position_m[0];
    message.position.y = result->position_m[1];
    message.position.z = result->position_m[2];
    message.ray_gap_m = static_cast<float>(result->ray_gap_m);
    result_publisher_->publish(message);
}

void StereoTriangulatorNode::publishFailure(const GreenLightDetection& left,
                                            const GreenLightDetection& right,
                                            const std::uint8_t status) {
    StereoTarget message;
    message.header = left.header;
    const auto l = rclcpp::Time(left.header.stamp).nanoseconds();
    const auto r = rclcpp::Time(right.header.stamp).nanoseconds();
    message.header.stamp = rclcpp::Time(l + (r - l) / 2);
    message.header.frame_id = reference_frame_;
    message.status = status;
    result_publisher_->publish(message);
}

std::optional<cv::Vec3d>
StereoTriangulatorNode::bearing(const GreenLightDetection& detection,
                                const std::deque<Info::ConstSharedPtr>& infos) {
    for (auto it = infos.rbegin(); it != infos.rend(); ++it) {
        const auto& info = **it;
        if (info.header.stamp != detection.header.stamp)
            continue;
        if (info.header.frame_id != detection.header.frame_id ||
            info.distortion_model != "plumb_bob" || info.width == 0 || info.height == 0 ||
            !std::isfinite(detection.center_u) || !std::isfinite(detection.center_v) ||
            detection.center_u < 0 || detection.center_v < 0 || detection.center_u >= info.width ||
            detection.center_v >= info.height || info.binning_x > 1 || info.binning_y > 1 ||
            info.roi.x_offset || info.roi.y_offset)
            return std::nullopt;
        BearingSolverConfig config;
        config.camera_matrix = info.k;
        config.distortion_coefficients = info.d;
        if (!config.isConfigValid())
            return std::nullopt;
        return BearingSolver(config).calculateUnitBearing({detection.center_u, detection.center_v});
    }
    return std::nullopt;
}
} // namespace dart_vision::stereo
