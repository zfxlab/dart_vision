#include "dart_aiming/aiming_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <limits>
#include <rclcpp/create_timer.hpp>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dart_vision::aiming {

AimingNode::AimingNode(const rclcpp::NodeOptions& options) : Node("aiming", options) {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "启动时读取，修改后需要重启节点";
    const auto stereo_topic =
        declare_parameter<std::string>("stereo_topic", "/camera/stereo_target", read_only);
    const auto controller_topic =
        declare_parameter<std::string>("controller_topic", "/controller_state", read_only);
    const auto command_topic =
        declare_parameter<std::string>("command_topic", "/aim_command", read_only);
    reference_frame_ =
        declare_parameter<std::string>("reference_frame", "launcher_frame", read_only);
    target_timeout_s_ = declare_parameter<double>("target_timeout_s", 0.2, read_only);
    controller_timeout_s_ = declare_parameter<double>("controller_timeout_s", 0.3, read_only);
    max_ray_gap_m_ = declare_parameter<double>("max_ray_gap_m", 0.1, read_only);
    const int frames = declare_parameter<int>("confirmation_frames", 3, read_only);
    const double yaw_step = declare_parameter<double>("max_yaw_step_rad", 0.02, read_only);
    const double distance_step = declare_parameter<double>("max_distance_step_m", 0.3, read_only);
    supported_target_modes_ = declare_parameter<std::vector<std::int64_t>>(
        "supported_target_modes", {1, 2, 3, 4}, read_only);
    const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
    if (reference_frame_.empty() || stereo_topic.empty() || controller_topic.empty() ||
        command_topic.empty() || !positive(target_timeout_s_) || !positive(controller_timeout_s_) ||
        !positive(max_ray_gap_m_))
        throw std::invalid_argument("Invalid aiming node configuration");
    stability_ = std::make_unique<Stability>(frames, yaw_step, distance_step);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<AimCommand>(command_topic, 10);
    controller_subscription_ = create_subscription<ControllerState>(
        controller_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&AimingNode::onController, this, std::placeholders::_1));
    target_subscription_ = create_subscription<StereoTarget>(
        stereo_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&AimingNode::onTarget, this, std::placeholders::_1));
    timer_ = rclcpp::create_timer(this,
                                  get_clock(),
                                  rclcpp::Duration::from_seconds(0.02),
                                  std::bind(&AimingNode::tick, this));
    RCLCPP_INFO(get_logger(), "Aiming output frame: '%s'", reference_frame_.c_str());
}

bool AimingNode::fresh(const builtin_interfaces::msg::Time& stamp, double timeout) const {
    const double age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= 0.0 && age <= timeout;
}

void AimingNode::invalidate(bool preserve_closed) {
    stability_->reset();
    if (preserve_closed && command_ && command_->state == AimCommand::CLOSED)
        return;
    command_.reset();
    target_stamp_.reset();
}

void AimingNode::onController(ControllerState::ConstSharedPtr message) {
    if (!fresh(message->header.stamp, controller_timeout_s_) ||
        !std::isfinite(message->launcher_yaw_rad) || !std::isfinite(message->dart_offset_rad) ||
        std::find(supported_target_modes_.begin(),
                  supported_target_modes_.end(),
                  message->target_mode) == supported_target_modes_.end()) {
        controller_.reset();
        invalidate(true);
        return;
    }
    if (controller_ &&
        rclcpp::Time(message->header.stamp) <= rclcpp::Time(controller_->header.stamp))
        return;
    if (!controller_ || !fresh(controller_->header.stamp, controller_timeout_s_) ||
        controller_->target_mode != message->target_mode ||
        controller_->dart_offset_rad != message->dart_offset_rad)
        invalidate(true);
    controller_ = std::move(message);
}

void AimingNode::onTarget(StereoTarget::ConstSharedPtr target) {
    if (!fresh(target->header.stamp, target_timeout_s_)) {
        invalidate();
        return;
    }
    if (last_target_stamp_) {
        if (rclcpp::Time(*last_target_stamp_) > now()) {
            // ROS 时钟回退时清除旧的去重时间。
            last_target_stamp_.reset();
            invalidate();
        } else if (rclcpp::Time(target->header.stamp) <= rclcpp::Time(*last_target_stamp_)) {
            return;
        }
    }
    last_target_stamp_ = target->header.stamp;
    if (target->header.frame_id.empty()) {
        invalidate();
        return;
    }
    if (target_stamp_ && !fresh(*target_stamp_, target_timeout_s_))
        invalidate();
    // CLOSED 是视觉状态，不需要控制器反馈或坐标变换。
    if (target->status == StereoTarget::CLOSED) {
        stability_->reset();
        command_.emplace();
        command_->state = AimCommand::CLOSED;
        target_stamp_ = target->header.stamp;
        tick();
        return;
    }
    if (target->status != StereoTarget::VALID || !std::isfinite(target->ray_gap_m) ||
        target->ray_gap_m < 0.0 || target->ray_gap_m > max_ray_gap_m_ || !controller_ ||
        !fresh(controller_->header.stamp, controller_timeout_s_)) {
        invalidate();
        return;
    }
    geometry_msgs::msg::PointStamped source, transformed;
    source.header = target->header;
    source.point = target->position;
    try {
        // 使用目标测量时刻的变换，不能以最新 TF 替代历史姿态。
        const auto transform = tf_buffer_->lookupTransform(
            reference_frame_, source.header.frame_id, rclcpp::Time(source.header.stamp));
        tf2::doTransform(source, transformed, transform);
    } catch (const tf2::TransformException& error) {
        invalidate();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Aiming transform unavailable: %s", error.what());
        return;
    }
    const auto& p = transformed.point;
    // 输出已相对发射架，不再次减去电机反馈角。
    const auto aim = solve(p.x, p.y, p.z, controller_->dart_offset_rad);
    if (!aim || aim->distance_m > std::numeric_limits<float>::max()) {
        invalidate();
        return;
    }
    AimCommand command;
    command.state = AimCommand::INVALID;
    if (stability_->update(*aim)) {
        command.state = AimCommand::VALID;
        command.yaw_error_rad = static_cast<float>(aim->yaw_error_rad);
        command.distance_m = static_cast<float>(aim->distance_m);
    }
    command_ = command;
    target_stamp_ = target->header.stamp;
    tick();
}

void AimingNode::tick() {
    if (!target_stamp_ || !fresh(*target_stamp_, target_timeout_s_) ||
        ((!command_ || command_->state != AimCommand::CLOSED) &&
         (!controller_ || !fresh(controller_->header.stamp, controller_timeout_s_))))
        invalidate();
    AimCommand message;
    message.state = AimCommand::INVALID;
    if (command_)
        message = *command_;
    message.header.stamp = now();
    message.header.frame_id = reference_frame_;
    publisher_->publish(message);
}
} // namespace dart_vision::aiming
