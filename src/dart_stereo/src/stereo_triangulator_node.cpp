#include "dart_stereo/stereo_triangulator_node.hpp"

#include <algorithm>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <functional>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>

namespace dart_vision::stereo {
namespace {
constexpr int kDebugThrottleMs = 1000;
constexpr int kWarningThrottleMs = 5000;
constexpr auto kDiagnosticsPeriod = std::chrono::seconds(1);
constexpr std::size_t kLatencyWindowSize = 200;

diagnostic_msgs::msg::KeyValue keyValue(const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = value;
    return result;
}

struct LatencySummary {
    double average{};
    double maximum{};
    double p95{};
};

LatencySummary summarizeLatency(const std::deque<double>& samples) {
    if (samples.empty())
        return {};
    std::vector<double> sorted(samples.begin(), samples.end());
    std::sort(sorted.begin(), sorted.end());
    double sum = 0.0;
    for (const double sample : sorted)
        sum += sample;
    const std::size_t p95_index = (sorted.size() * 95U + 99U) / 100U - 1U;
    return {sum / static_cast<double>(sorted.size()), sorted.back(), sorted[p95_index]};
}

double stampSeconds(const std_msgs::msg::Header& header) {
    return rclcpp::Time(header.stamp).seconds();
}

std::optional<cv::Vec3d> bearing(const dart_interfaces::msg::GreenLightDetection& detection) {
    const auto& ray = detection.unit_ray;
    const double norm = std::hypot(ray.x, ray.y, ray.z);
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-6 || ray.z <= 0.0)
        return std::nullopt;
    return cv::Vec3d{ray.x, ray.y, ray.z};
}

} // namespace

StereoTriangulatorNode::StereoTriangulatorNode(const rclcpp::NodeOptions& options)
    : Node("stereo_triangulator", options),
      previous_diagnostic_time_(std::chrono::steady_clock::now()) {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "Loaded at startup; restart the node to change this parameter";

    const std::string top_topic =
        declare_parameter<std::string>("top_detection_topic", "/top_camera/detection", read_only);
    const std::string bottom_topic = declare_parameter<std::string>(
        "bottom_detection_topic", "/bottom_camera/detection", read_only);
    const std::string result_topic =
        declare_parameter<std::string>("result_topic", "/stereo_target", read_only);
    top_frame_id_ =
        declare_parameter<std::string>("top_frame_id", "top_camera_optical_frame", read_only);
    bottom_frame_id_ =
        declare_parameter<std::string>("bottom_frame_id", "bottom_camera_optical_frame", read_only);
    reference_frame_ =
        declare_parameter<std::string>("reference_frame", "launcher_frame", read_only);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    rcl_jump_threshold_t threshold{};
    threshold.min_backward.nanoseconds = -1;
    threshold.on_clock_change = true;
    jump_handler_ = get_clock()->create_jump_callback(
        nullptr, [this](const rcl_time_jump_t&) { clock_jumped_.store(true); }, threshold);
    max_pair_delta_s_ = declare_parameter<double>("max_pair_delta_s", 0.03, read_only);
    const std::int64_t configured_queue_size = declare_parameter<int>("queue_size", 10, read_only);

    StereoTriangulatorConfig config;
    config.min_ray_angle_deg =
        declare_parameter<double>("min_ray_angle_deg", config.min_ray_angle_deg, read_only);
    config.min_depth_m = declare_parameter<double>("min_depth_m", config.min_depth_m, read_only);
    config.max_camera_range_m =
        declare_parameter<double>("max_camera_range_m", config.max_camera_range_m, read_only);
    config.max_ray_gap_m =
        declare_parameter<double>("max_ray_gap_m", config.max_ray_gap_m, read_only);

    if (top_topic.empty() || bottom_topic.empty() || result_topic.empty() ||
        top_frame_id_.empty() || bottom_frame_id_.empty() || reference_frame_.empty() ||
        !std::isfinite(max_pair_delta_s_) || max_pair_delta_s_ < 0.0 ||
        configured_queue_size <= 0 || top_frame_id_ == bottom_frame_id_) {
        throw std::invalid_argument("Invalid stereo triangulator node configuration");
    }
    queue_size_ = static_cast<std::size_t>(configured_queue_size);
    triangulator_ = std::make_unique<StereoTriangulator>(config);

    result_publisher_ = create_publisher<StereoTarget>(result_topic, rclcpp::SensorDataQoS());
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
    diagnostics_timer_ = create_wall_timer(
        kDiagnosticsPeriod, std::bind(&StereoTriangulatorNode::publishDiagnostics, this));
    top_subscription_ = create_subscription<GreenLightDetection>(
        top_topic, rclcpp::SensorDataQoS(), [this](GreenLightDetection::ConstSharedPtr message) {
            observationCallback(message, true);
        });
    bottom_subscription_ = create_subscription<GreenLightDetection>(
        bottom_topic, rclcpp::SensorDataQoS(), [this](GreenLightDetection::ConstSharedPtr message) {
            observationCallback(message, false);
        });

    RCLCPP_INFO(get_logger(), "Stereo triangulator: top='%s', bottom='%s', output='%s'",
                top_topic.c_str(), bottom_topic.c_str(), result_topic.c_str());
}

void StereoTriangulatorNode::observationCallback(const GreenLightDetection::ConstSharedPtr& message,
                                                 const bool is_top) {
    {
        std::lock_guard<std::mutex> lock(diagnostic_mutex_);
        if (is_top) {
            ++diagnostic_statistics_.top_received_total;
            ++diagnostic_statistics_.top_received_interval;
        } else {
            ++diagnostic_statistics_.bottom_received_total;
            ++diagnostic_statistics_.bottom_received_interval;
        }
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    auto& queue = is_top ? top_queue_ : bottom_queue_;
    // Clear on an actual ROS clock jump, not on reordered observations.
    if (clock_jumped_.exchange(false)) {
        top_queue_.clear();
        bottom_queue_.clear();
    }
    queue.push_back(message);
    while (queue.size() > queue_size_) {
        queue.pop_front();
        std::lock_guard<std::mutex> diagnostic_lock(diagnostic_mutex_);
        if (is_top)
            ++diagnostic_statistics_.unmatched_top_total;
        else
            ++diagnostic_statistics_.unmatched_bottom_total;
    }
    matchQueuedObservations();
}

void StereoTriangulatorNode::matchQueuedObservations() {
    while (!top_queue_.empty() && !bottom_queue_.empty()) {
        std::size_t best_top = 0U;
        std::size_t best_bottom = 0U;
        double best_delta = std::numeric_limits<double>::infinity();
        for (std::size_t top_index = 0; top_index < top_queue_.size(); ++top_index) {
            for (std::size_t bottom_index = 0; bottom_index < bottom_queue_.size();
                 ++bottom_index) {
                const double delta = std::abs(stampSeconds(top_queue_[top_index]->header) -
                                              stampSeconds(bottom_queue_[bottom_index]->header));
                if (delta < best_delta) {
                    best_delta = delta;
                    best_top = top_index;
                    best_bottom = bottom_index;
                }
            }
        }

        if (best_delta <= max_pair_delta_s_) {
            const auto top = top_queue_[best_top];
            const auto bottom = bottom_queue_[best_bottom];
            {
                std::lock_guard<std::mutex> lock(diagnostic_mutex_);
                diagnostic_statistics_.unmatched_top_total += best_top;
                diagnostic_statistics_.unmatched_bottom_total += best_bottom;
            }
            top_queue_.erase(top_queue_.begin(), top_queue_.begin() + best_top + 1U);
            bottom_queue_.erase(bottom_queue_.begin(), bottom_queue_.begin() + best_bottom + 1U);
            processPair(*top, *bottom);
            continue;
        }

        const double top_stamp = stampSeconds(top_queue_.front()->header);
        const double bottom_stamp = stampSeconds(bottom_queue_.front()->header);
        if (top_stamp + max_pair_delta_s_ < bottom_stamp) {
            top_queue_.pop_front();
            std::lock_guard<std::mutex> lock(diagnostic_mutex_);
            ++diagnostic_statistics_.unmatched_top_total;
        } else if (bottom_stamp + max_pair_delta_s_ < top_stamp) {
            bottom_queue_.pop_front();
            std::lock_guard<std::mutex> lock(diagnostic_mutex_);
            ++diagnostic_statistics_.unmatched_bottom_total;
        } else {
            break;
        }
        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), kDebugThrottleMs,
            "Dropping unmatched stereo observation; closest timestamp delta %.4f s", best_delta);
    }
}

void StereoTriangulatorNode::processPair(const GreenLightDetection& top,
                                         const GreenLightDetection& bottom) {
    const double pair_delta_ms =
        std::abs(stampSeconds(top.header) - stampSeconds(bottom.header)) * 1000.0;
    {
        std::lock_guard<std::mutex> lock(diagnostic_mutex_);
        ++diagnostic_statistics_.pairs_total;
        ++diagnostic_statistics_.pairs_interval;
        diagnostic_statistics_.pair_delta_ms.push_back(pair_delta_ms);
        if (diagnostic_statistics_.pair_delta_ms.size() > kLatencyWindowSize)
            diagnostic_statistics_.pair_delta_ms.pop_front();
    }
    if (top.header.frame_id != top_frame_id_ || bottom.header.frame_id != bottom_frame_id_) {
        {
            std::lock_guard<std::mutex> lock(diagnostic_mutex_);
            ++diagnostic_statistics_.frame_mismatch_total;
        }
        publishFailure(top, bottom, StereoTarget::INVALID);
        return;
    }
    if (top.status == GreenLightDetection::CLOSED && bottom.status == GreenLightDetection::CLOSED) {
        publishFailure(top, bottom, StereoTarget::CLOSED);
        return;
    }
    if (top.status != GreenLightDetection::DETECTED) {
        {
            std::lock_guard<std::mutex> lock(diagnostic_mutex_);
            ++diagnostic_statistics_.input_status_invalid_total;
        }
        publishFailure(top, bottom, StereoTarget::INVALID);
        return;
    }
    if (bottom.status != GreenLightDetection::DETECTED) {
        {
            std::lock_guard<std::mutex> lock(diagnostic_mutex_);
            ++diagnostic_statistics_.input_status_invalid_total;
        }
        publishFailure(top, bottom, StereoTarget::INVALID);
        return;
    }

    const auto lb = bearing(top);
    const auto rb = bearing(bottom);
    if (!lb || !rb) {
        {
            std::lock_guard<std::mutex> lock(diagnostic_mutex_);
            ++diagnostic_statistics_.bearing_invalid_total;
        }
        publishFailure(top, bottom, StereoTarget::INVALID);
        return;
    }
    const auto l = rclcpp::Time(top.header.stamp).nanoseconds();
    const auto r = rclcpp::Time(bottom.header.stamp).nanoseconds();
    const rclcpp::Time measurement_time(l + (r - l) / 2);
    cv::Vec3d top_origin, bottom_origin, top_direction, bottom_direction;
    try {
        // 光心使用 TF 的平移，视线是自由向量，仅使用 TF 的旋转。
        const auto transform_ray = [&](const std::string& frame, const cv::Vec3d& bearing,
                                       cv::Vec3d& origin, cv::Vec3d& direction) {
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
        transform_ray(top_frame_id_, *lb, top_origin, top_direction);
        transform_ray(bottom_frame_id_, *rb, bottom_origin, bottom_direction);
    } catch (const tf2::TransformException& error) {
        {
            std::lock_guard<std::mutex> lock(diagnostic_mutex_);
            ++diagnostic_statistics_.transform_errors_total;
            ++diagnostic_statistics_.transform_errors_interval;
        }
        publishFailure(top, bottom, StereoTarget::INVALID);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarningThrottleMs,
                             "Stereo reference transform unavailable: %s", error.what());
        return;
    }
    StereoTriangulationDiagnostics diagnostics;
    const auto result = triangulator_->triangulate(top_origin, top_direction, bottom_origin,
                                                   bottom_direction, &diagnostics);
    if (!result) {
        {
            std::lock_guard<std::mutex> lock(diagnostic_mutex_);
            const auto index = static_cast<std::size_t>(diagnostics.rejection);
            if (index < diagnostic_statistics_.rejection_totals.size())
                ++diagnostic_statistics_.rejection_totals[index];
        }
        publishFailure(top, bottom, StereoTarget::INVALID);
        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), kDebugThrottleMs,
            "Stereo triangulation rejected: reason=%s, pair_delta=%.6f s, "
            "baseline=%.6f m, "
            "ray_angle=%.6f deg, top_depth=%.6f m, bottom_depth=%.6f m, "
            "ray_gap=%.6f m, camera_range=%.6f m, midpoint_x=%.6f m, "
            "top_dir=[%.6f, %.6f, %.6f], bottom_dir=[%.6f, %.6f, %.6f]",
            stereoTriangulationRejectionName(diagnostics.rejection),
            std::abs(stampSeconds(top.header) - stampSeconds(bottom.header)),
            diagnostics.baseline_m, diagnostics.ray_angle_deg, diagnostics.top_distance_m,
            diagnostics.bottom_distance_m, diagnostics.ray_gap_m, diagnostics.camera_range_m,
            diagnostics.midpoint_x_m, top_direction[0], top_direction[1], top_direction[2],
            bottom_direction[0], bottom_direction[1], bottom_direction[2]);
        return;
    }

    StereoTarget message;
    message.header = top.header;
    message.header.stamp = measurement_time;
    message.header.frame_id = reference_frame_;
    message.status = StereoTarget::VALID;
    message.position.x = result->position_m[0];
    message.position.y = result->position_m[1];
    message.position.z = result->position_m[2];
    message.distance = result->distance_m;
    message.yaw = std::atan2(message.position.y, message.position.x);
    message.ray_gap_m = static_cast<float>(result->ray_gap_m);
    result_publisher_->publish(message);
    recordOutput(message.status, rclcpp::Time(message.header.stamp));
}

void StereoTriangulatorNode::publishFailure(const GreenLightDetection& top,
                                            const GreenLightDetection& bottom,
                                            const std::uint8_t status) {
    StereoTarget message;
    message.header = top.header;
    const auto l = rclcpp::Time(top.header.stamp).nanoseconds();
    const auto r = rclcpp::Time(bottom.header.stamp).nanoseconds();
    message.header.stamp = rclcpp::Time(l + (r - l) / 2);
    message.header.frame_id = reference_frame_;
    message.status = status;
    result_publisher_->publish(message);
    recordOutput(message.status, rclcpp::Time(message.header.stamp));
}

void StereoTriangulatorNode::recordOutput(const std::uint8_t status, const rclcpp::Time& stamp) {
    const double latency_ms = (now() - stamp).seconds() * 1000.0;
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    if (status == StereoTarget::VALID) {
        ++diagnostic_statistics_.valid_total;
        ++diagnostic_statistics_.valid_interval;
    } else if (status == StereoTarget::CLOSED) {
        ++diagnostic_statistics_.closed_total;
    } else {
        ++diagnostic_statistics_.invalid_total;
    }
    if (std::isfinite(latency_ms) && latency_ms >= 0.0) {
        diagnostic_statistics_.measurement_to_stereo_ms.push_back(latency_ms);
        if (diagnostic_statistics_.measurement_to_stereo_ms.size() > kLatencyWindowSize)
            diagnostic_statistics_.measurement_to_stereo_ms.pop_front();
    }
    diagnostic_statistics_.last_output_time = std::chrono::steady_clock::now();
}

void StereoTriangulatorNode::publishDiagnostics() {
    const auto current_time = std::chrono::steady_clock::now();
    const double interval_seconds =
        std::chrono::duration<double>(current_time - previous_diagnostic_time_).count();
    previous_diagnostic_time_ = current_time;

    DiagnosticStatistics statistics;
    {
        std::lock_guard<std::mutex> lock(diagnostic_mutex_);
        statistics = diagnostic_statistics_;
        diagnostic_statistics_.top_received_interval = 0;
        diagnostic_statistics_.bottom_received_interval = 0;
        diagnostic_statistics_.pairs_interval = 0;
        diagnostic_statistics_.valid_interval = 0;
        diagnostic_statistics_.transform_errors_interval = 0;
    }
    const auto rate = [interval_seconds](const std::uint64_t count) {
        return interval_seconds > 0.0 ? static_cast<double>(count) / interval_seconds : 0.0;
    };
    const auto pair_delta = summarizeLatency(statistics.pair_delta_ms);
    const auto latency = summarizeLatency(statistics.measurement_to_stereo_ms);
    const double last_output_age =
        statistics.last_output_time == std::chrono::steady_clock::time_point{}
            ? -1.0
            : std::chrono::duration<double>(current_time - statistics.last_output_time).count();

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_fully_qualified_name()) + ": stereo_triangulator";
    status.hardware_id = "none";
    if (statistics.top_received_interval == 0 || statistics.bottom_received_interval == 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "missing stereo input";
    } else if (statistics.pairs_interval == 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "no stereo pairs";
    } else if (statistics.transform_errors_interval > 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "transform errors";
    } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "processing";
    }
    status.values.push_back(
        keyValue("top_input_fps", std::to_string(rate(statistics.top_received_interval))));
    status.values.push_back(
        keyValue("bottom_input_fps", std::to_string(rate(statistics.bottom_received_interval))));
    status.values.push_back(
        keyValue("paired_fps", std::to_string(rate(statistics.pairs_interval))));
    status.values.push_back(keyValue("valid_fps", std::to_string(rate(statistics.valid_interval))));
    status.values.push_back(keyValue("pair_delta_average_ms", std::to_string(pair_delta.average)));
    status.values.push_back(keyValue("pair_delta_max_ms", std::to_string(pair_delta.maximum)));
    status.values.push_back(keyValue("pair_delta_p95_ms", std::to_string(pair_delta.p95)));
    status.values.push_back(
        keyValue("measurement_to_stereo_average_ms", std::to_string(latency.average)));
    status.values.push_back(
        keyValue("measurement_to_stereo_max_ms", std::to_string(latency.maximum)));
    status.values.push_back(keyValue("measurement_to_stereo_p95_ms", std::to_string(latency.p95)));
    status.values.push_back(keyValue("last_output_age_sec", std::to_string(last_output_age)));
    status.values.push_back(keyValue("pairs_total", std::to_string(statistics.pairs_total)));
    status.values.push_back(keyValue("valid_total", std::to_string(statistics.valid_total)));
    status.values.push_back(keyValue("closed_total", std::to_string(statistics.closed_total)));
    status.values.push_back(keyValue("invalid_total", std::to_string(statistics.invalid_total)));
    status.values.push_back(
        keyValue("unmatched_top_total", std::to_string(statistics.unmatched_top_total)));
    status.values.push_back(
        keyValue("unmatched_bottom_total", std::to_string(statistics.unmatched_bottom_total)));
    status.values.push_back(
        keyValue("top_received_total", std::to_string(statistics.top_received_total)));
    status.values.push_back(
        keyValue("bottom_received_total", std::to_string(statistics.bottom_received_total)));
    status.values.push_back(
        keyValue("frame_mismatch_total", std::to_string(statistics.frame_mismatch_total)));
    status.values.push_back(keyValue("input_status_invalid_total",
                                     std::to_string(statistics.input_status_invalid_total)));
    status.values.push_back(
        keyValue("bearing_invalid_total", std::to_string(statistics.bearing_invalid_total)));
    status.values.push_back(
        keyValue("transform_errors_total", std::to_string(statistics.transform_errors_total)));
    for (std::size_t index = 1; index < statistics.rejection_totals.size(); ++index) {
        const auto rejection = static_cast<StereoTriangulationRejection>(index);
        status.values.push_back(keyValue(std::string("rejected_") +
                                             stereoTriangulationRejectionName(rejection) + "_total",
                                         std::to_string(statistics.rejection_totals[index])));
    }
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
}

} // namespace dart_vision::stereo
