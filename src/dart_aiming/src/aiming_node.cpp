#include "dart_aiming/aiming_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/create_timer.hpp>
#include <regex>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_set>
#include <vector>

namespace dart_vision::aiming {
namespace {
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

bool validModelName(const std::string& name) {
    static const std::regex pattern{"[A-Za-z][A-Za-z0-9_]*"};
    return std::regex_match(name, pattern);
}

QuadraticAimModel::Coefficients coefficientsFromParameter(const std::vector<double>& values,
                                                          const std::string& parameter_name) {
    if (values.size() != 3U)
        throw std::invalid_argument(parameter_name + " must contain exactly [a, b, c]");
    return {values[0], values[1], values[2]};
}
} // namespace

AimingNode::AimingNode(const rclcpp::NodeOptions& options)
    : Node("aiming", options), previous_diagnostic_time_(std::chrono::steady_clock::now()) {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "启动时读取，修改后需要重启节点";
    const auto stereo_topic =
        declare_parameter<std::string>("stereo_topic", "/stereo_target", read_only);
    const auto controller_topic =
        declare_parameter<std::string>("controller_topic", "/controller_state", read_only);
    const auto command_topic =
        declare_parameter<std::string>("command_topic", "/aim_command", read_only);
    reference_frame_ =
        declare_parameter<std::string>("reference_frame", "launcher_frame", read_only);
    target_timeout_s_ = declare_parameter<double>("target_timeout_s", 0.2, read_only);
    controller_timeout_s_ = declare_parameter<double>("controller_timeout_s", 0.3, read_only);
    max_height_gap_m_ = declare_parameter<double>("max_height_gap_m", 0.1, read_only);
    supported_target_modes_ = declare_parameter<std::vector<std::int64_t>>("supported_target_modes",
                                                                           {1, 2, 3, 4}, read_only);
    const auto fitting_model_names = declare_parameter<std::vector<std::string>>(
        "fitting.model_names", std::vector<std::string>{}, read_only);
    const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
    if (reference_frame_.empty() || stereo_topic.empty() || controller_topic.empty() ||
        command_topic.empty() || !positive(target_timeout_s_) || !positive(controller_timeout_s_) ||
        !positive(max_height_gap_m_))
        throw std::invalid_argument("Invalid aiming node configuration");

    const std::unordered_set<std::int64_t> supported_modes(supported_target_modes_.begin(),
                                                           supported_target_modes_.end());
    if (supported_modes.size() != supported_target_modes_.size() ||
        std::any_of(supported_target_modes_.begin(), supported_target_modes_.end(),
                    [](const std::int64_t mode) { return mode < 0 || mode > 255; }))
        throw std::invalid_argument("supported_target_modes must contain unique uint8 values");

    std::unordered_set<std::string> loaded_model_names;
    for (const auto& model_name : fitting_model_names) {
        if (!validModelName(model_name) || !loaded_model_names.insert(model_name).second)
            throw std::invalid_argument("Invalid or duplicate fitting model name: " + model_name);
        const std::string prefix = "fitting.models." + model_name + ".";
        const auto modes = declare_parameter<std::vector<std::int64_t>>(
            prefix + "modes", std::vector<std::int64_t>{}, read_only);
        const auto yaw_coefficients =
            coefficientsFromParameter(declare_parameter<std::vector<double>>(
                                          prefix + "yaw_coefficients", {0.0, 1.0, 0.0}, read_only),
                                      prefix + "yaw_coefficients");
        const auto distance_coefficients = coefficientsFromParameter(
            declare_parameter<std::vector<double>>(prefix + "distance_coefficients",
                                                   {0.0, 1.0, 0.0}, read_only),
            prefix + "distance_coefficients");
        if (modes.empty())
            throw std::invalid_argument("Fitting model has no target modes: " + model_name);
        const QuadraticAimModel model(yaw_coefficients, distance_coefficients);
        std::unordered_set<std::int64_t> model_modes;
        for (const auto mode : modes) {
            if (!supported_modes.count(mode) || !model_modes.insert(mode).second)
                throw std::invalid_argument("Invalid or duplicate target mode in model: " +
                                            model_name);
            if (!fitting_models_.emplace(static_cast<std::uint8_t>(mode), model).second)
                throw std::invalid_argument("Target mode belongs to multiple fitting models: " +
                                            std::to_string(mode));
        }
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<AimCommand>(command_topic, 10);
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
    diagnostics_timer_ =
        create_wall_timer(kDiagnosticsPeriod, std::bind(&AimingNode::publishDiagnostics, this));
    controller_subscription_ = create_subscription<ControllerState>(
        controller_topic, rclcpp::SensorDataQoS(),
        std::bind(&AimingNode::onController, this, std::placeholders::_1));
    target_subscription_ = create_subscription<StereoTarget>(
        stereo_topic, rclcpp::SensorDataQoS(),
        std::bind(&AimingNode::onTarget, this, std::placeholders::_1));
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.02),
                                  std::bind(&AimingNode::tick, this));
    RCLCPP_INFO(get_logger(), "Aiming output frame: '%s'", reference_frame_.c_str());
}

bool AimingNode::fresh(const builtin_interfaces::msg::Time& stamp, double timeout) const {
    const double age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= 0.0 && age <= timeout;
}

void AimingNode::invalidate(bool preserve_closed) {
    if (preserve_closed && command_ && command_->state == AimCommand::CLOSED)
        return;
    command_.reset();
    target_stamp_.reset();
}

void AimingNode::onController(ControllerState::ConstSharedPtr message) {
    if (!fresh(message->header.stamp, controller_timeout_s_) ||
        !std::isfinite(message->launcher_yaw_rad) || !std::isfinite(message->dart_offset_rad) ||
        std::find(supported_target_modes_.begin(), supported_target_modes_.end(),
                  message->target_mode) == supported_target_modes_.end()) {
        const bool invalidates_active_command = command_ && command_->state != AimCommand::CLOSED;
        controller_.reset();
        invalidate(true);
        if (invalidates_active_command)
            recordInvalid(InvalidReason::controller_unavailable);
        return;
    }
    if (controller_ &&
        rclcpp::Time(message->header.stamp) <= rclcpp::Time(controller_->header.stamp))
        return;
    if (!controller_ || !fresh(controller_->header.stamp, controller_timeout_s_) ||
        controller_->target_mode != message->target_mode ||
        controller_->dart_offset_rad != message->dart_offset_rad) {
        const bool invalidates_active_command = command_ && command_->state != AimCommand::CLOSED;
        invalidate(true);
        if (invalidates_active_command)
            recordInvalid(InvalidReason::controller_changed);
    }
    controller_ = std::move(message);
}

void AimingNode::onTarget(StereoTarget::ConstSharedPtr target) {
    {
        std::lock_guard<std::mutex> lock(diagnostic_mutex_);
        ++diagnostic_statistics_.targets_received_total;
        ++diagnostic_statistics_.targets_received_interval;
        diagnostic_statistics_.last_target_time = std::chrono::steady_clock::now();
    }
    if (!fresh(target->header.stamp, target_timeout_s_)) {
        invalidate();
        recordInvalid(InvalidReason::stale_target);
        return;
    }
    if (last_target_stamp_) {
        if (rclcpp::Time(*last_target_stamp_) > now()) {
            // ROS 时钟回退时清除旧的去重时间。
            last_target_stamp_.reset();
            invalidate();
            recordInvalid(InvalidReason::clock_regression);
        } else if (rclcpp::Time(target->header.stamp) <= rclcpp::Time(*last_target_stamp_)) {
            std::lock_guard<std::mutex> lock(diagnostic_mutex_);
            ++diagnostic_statistics_.duplicate_total;
            return;
        }
    }
    last_target_stamp_ = target->header.stamp;
    if (target->header.frame_id.empty()) {
        invalidate();
        recordInvalid(InvalidReason::invalid_frame);
        return;
    }
    if (target_stamp_ && !fresh(*target_stamp_, target_timeout_s_))
        invalidate();
    // CLOSED 是视觉状态，不需要控制器反馈或坐标变换。
    if (target->status == StereoTarget::CLOSED) {
        invalidate();
        command_.emplace();
        command_->header.stamp = target->header.stamp;
        command_->header.frame_id = reference_frame_;
        command_->state = AimCommand::CLOSED;
        target_stamp_ = target->header.stamp;
        recordAccepted(AimCommand::CLOSED, target->header.stamp);
        tick();
        return;
    }
    if (target->status != StereoTarget::VALID || !std::isfinite(target->height_gap_m) ||
        target->height_gap_m < 0.0 || target->height_gap_m > max_height_gap_m_) {
        invalidate();
        recordInvalid(InvalidReason::invalid_target);
        return;
    }
    if (!controller_ || !fresh(controller_->header.stamp, controller_timeout_s_)) {
        invalidate();
        recordInvalid(InvalidReason::controller_unavailable);
        return;
    }
    geometry_msgs::msg::PointStamped source, transformed;
    source.header = target->header;
    source.point = target->position;
    try {
        // 使用目标测量时刻的变换，不能以最新 TF 替代历史姿态。
        const auto transform = tf_buffer_->lookupTransform(reference_frame_, source.header.frame_id,
                                                           rclcpp::Time(source.header.stamp));
        tf2::doTransform(source, transformed, transform);
    } catch (const tf2::TransformException& error) {
        invalidate();
        recordInvalid(InvalidReason::transform_unavailable);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarningThrottleMs,
                             "Aiming transform unavailable: %s", error.what());
        return;
    }
    const auto& p = transformed.point;
    // 输出已位于配置的参考坐标系，不再次减去电机反馈角。
    const auto geometric_aim = solve(p.x, p.y, p.z);
    if (!geometric_aim) {
        invalidate();
        recordInvalid(InvalidReason::geometry_invalid);
        return;
    }
    std::optional<Aim> modeled_aim = geometric_aim;
    const auto model = fitting_models_.find(controller_->target_mode);
    if (model != fitting_models_.end())
        modeled_aim = model->second.apply(*geometric_aim);
    if (!modeled_aim) {
        invalidate();
        recordInvalid(InvalidReason::model_invalid);
        return;
    }
    const auto final_aim = applyDartOffset(*modeled_aim, controller_->dart_offset_rad);
    if (!final_aim || final_aim->distance_m > std::numeric_limits<float>::max()) {
        invalidate();
        recordInvalid(InvalidReason::offset_invalid);
        return;
    }
    AimCommand command;
    command.header.stamp = target->header.stamp;
    command.header.frame_id = reference_frame_;
    command.state = AimCommand::VALID;
    command.yaw_error_rad = static_cast<float>(final_aim->yaw_error_rad);
    command.distance_m = static_cast<float>(final_aim->distance_m);
    command_ = command;
    target_stamp_ = target->header.stamp;
    recordAccepted(AimCommand::VALID, target->header.stamp);
    tick();
}

void AimingNode::tick() {
    if (target_stamp_ && !fresh(*target_stamp_, target_timeout_s_)) {
        const bool had_command = command_.has_value();
        invalidate();
        if (had_command)
            recordInvalid(InvalidReason::target_timeout);
    } else if (command_ && command_->state != AimCommand::CLOSED &&
               (!controller_ || !fresh(controller_->header.stamp, controller_timeout_s_))) {
        invalidate();
        recordInvalid(InvalidReason::controller_timeout);
    }
    AimCommand message;
    message.state = AimCommand::INVALID;
    if (command_) {
        message = *command_;
    } else {
        if (last_target_stamp_)
            message.header.stamp = *last_target_stamp_;
        message.header.frame_id = reference_frame_;
    }
    publisher_->publish(message);
}

void AimingNode::recordInvalid(const InvalidReason reason) {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    diagnostic_statistics_.current_output_state = AimCommand::INVALID;
    diagnostic_statistics_.last_invalid_reason = reason;
    const auto index = static_cast<std::size_t>(reason);
    if (index < diagnostic_statistics_.invalid_reason_totals.size())
        ++diagnostic_statistics_.invalid_reason_totals[index];
}

void AimingNode::recordAccepted(const std::uint8_t state,
                                const builtin_interfaces::msg::Time& stamp) {
    const double latency_ms = (now() - rclcpp::Time(stamp)).seconds() * 1000.0;
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    diagnostic_statistics_.current_output_state = state;
    diagnostic_statistics_.last_invalid_reason = InvalidReason::none;
    if (state == AimCommand::VALID)
        ++diagnostic_statistics_.valid_total;
    else if (state == AimCommand::CLOSED)
        ++diagnostic_statistics_.closed_total;
    if (std::isfinite(latency_ms) && latency_ms >= 0.0) {
        diagnostic_statistics_.measurement_to_aim_ms.push_back(latency_ms);
        if (diagnostic_statistics_.measurement_to_aim_ms.size() > kLatencyWindowSize)
            diagnostic_statistics_.measurement_to_aim_ms.pop_front();
    }
}

void AimingNode::publishDiagnostics() {
    static constexpr std::array<const char*, static_cast<std::size_t>(InvalidReason::count)>
        reason_names{"none",
                     "waiting_for_target",
                     "stale_target",
                     "clock_regression",
                     "invalid_frame",
                     "invalid_target",
                     "controller_unavailable",
                     "controller_changed",
                     "transform_unavailable",
                     "geometry_invalid",
                     "model_invalid",
                     "offset_invalid",
                     "target_timeout",
                     "controller_timeout"};
    const auto current_time = std::chrono::steady_clock::now();
    const double interval_seconds =
        std::chrono::duration<double>(current_time - previous_diagnostic_time_).count();
    previous_diagnostic_time_ = current_time;
    DiagnosticStatistics statistics;
    {
        std::lock_guard<std::mutex> lock(diagnostic_mutex_);
        statistics = diagnostic_statistics_;
        diagnostic_statistics_.targets_received_interval = 0;
    }
    const double input_fps =
        interval_seconds > 0.0
            ? static_cast<double>(statistics.targets_received_interval) / interval_seconds
            : 0.0;
    const auto latency = summarizeLatency(statistics.measurement_to_aim_ms);
    const double last_target_age =
        statistics.last_target_time == std::chrono::steady_clock::time_point{}
            ? -1.0
            : std::chrono::duration<double>(current_time - statistics.last_target_time).count();
    const auto reason_index = static_cast<std::size_t>(statistics.last_invalid_reason);

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_fully_qualified_name()) + ": aiming";
    status.hardware_id = "none";
    if (statistics.current_output_state == AimCommand::INVALID) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = reason_names[reason_index];
    } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = statistics.current_output_state == AimCommand::VALID ? "valid" : "closed";
    }
    status.values.push_back(keyValue("target_input_fps", std::to_string(input_fps)));
    status.values.push_back(
        keyValue("current_output_state", std::to_string(statistics.current_output_state)));
    status.values.push_back(keyValue("last_invalid_reason", reason_names[reason_index]));
    status.values.push_back(
        keyValue("measurement_to_aim_average_ms", std::to_string(latency.average)));
    status.values.push_back(keyValue("measurement_to_aim_max_ms", std::to_string(latency.maximum)));
    status.values.push_back(keyValue("measurement_to_aim_p95_ms", std::to_string(latency.p95)));
    status.values.push_back(keyValue("last_target_age_sec", std::to_string(last_target_age)));
    status.values.push_back(
        keyValue("targets_received_total", std::to_string(statistics.targets_received_total)));
    status.values.push_back(keyValue("valid_total", std::to_string(statistics.valid_total)));
    status.values.push_back(keyValue("closed_total", std::to_string(statistics.closed_total)));
    status.values.push_back(
        keyValue("duplicate_total", std::to_string(statistics.duplicate_total)));
    for (std::size_t index = 1; index < statistics.invalid_reason_totals.size(); ++index) {
        status.values.push_back(keyValue(std::string("invalid_") + reason_names[index] + "_total",
                                         std::to_string(statistics.invalid_reason_totals[index])));
    }
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
}
} // namespace dart_vision::aiming
