#include "dart_aiming/aiming_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <iomanip>
#include <limits>
#include <rclcpp/create_timer.hpp>
#include <sstream>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
    enable_visualization_ = declare_parameter<bool>("enable_visualization", false, read_only);
    const auto visualization_topic =
        declare_parameter<std::string>("visualization_topic", "/aiming/markers", read_only);
    marker_lifetime_s_ = declare_parameter<double>("marker_lifetime_s", 0.3, read_only);
    target_marker_radius_m_ = declare_parameter<double>("target_marker_radius_m", 0.08, read_only);
    marker_line_width_m_ = declare_parameter<double>("marker_line_width_m", 0.02, read_only);
    reference_frame_ =
        declare_parameter<std::string>("reference_frame", "launcher_frame", read_only);
    target_timeout_s_ = declare_parameter<double>("target_timeout_s", 0.2, read_only);
    controller_timeout_s_ = declare_parameter<double>("controller_timeout_s", 0.3, read_only);
    const int frames = declare_parameter<int>("confirmation_frames", 3, read_only);
    const double yaw_step = declare_parameter<double>("max_yaw_step_rad", 0.02, read_only);
    const double distance_step = declare_parameter<double>("max_distance_step_m", 0.3, read_only);
    supported_target_modes_ = declare_parameter<std::vector<std::int64_t>>(
        "supported_target_modes", {1, 2, 3, 4}, read_only);
    const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
    if (reference_frame_.empty() || stereo_topic.empty() || controller_topic.empty() ||
        command_topic.empty() || !positive(target_timeout_s_) || !positive(controller_timeout_s_) ||
        (enable_visualization_ &&
         (visualization_topic.empty() || !positive(marker_lifetime_s_) ||
          !positive(target_marker_radius_m_) || !positive(marker_line_width_m_))))
        throw std::invalid_argument("Invalid aiming node configuration");
    stability_ = std::make_unique<Stability>(frames, yaw_step, distance_step);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<AimCommand>(command_topic, 10);
    if (enable_visualization_)
        visualization_publisher_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, 10);
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
    if (enable_visualization_)
        RCLCPP_INFO(get_logger(), "Aiming visualization topic: '%s'", visualization_topic.c_str());
}

bool AimingNode::fresh(const builtin_interfaces::msg::Time& stamp, double timeout) const {
    const double age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= 0.0 && age <= timeout;
}

void AimingNode::invalidate(bool preserve_closed) {
    stability_->reset();
    clearVisualization();
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
        clearVisualization();
        command_.emplace();
        command_->state = AimCommand::CLOSED;
        target_stamp_ = target->header.stamp;
        tick();
        return;
    }
    if (target->status != StereoTarget::VALID || !controller_ ||
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
    // 输出已位于配置的参考坐标系，不再次减去电机反馈角。
    const auto geometric_aim = solve(p.x, p.y, p.z);
    if (!geometric_aim) {
        invalidate();
        return;
    }
    const auto fitted_aim = applyFittedCorrection(*geometric_aim);
    if (!fitted_aim) {
        invalidate();
        return;
    }
    const auto final_aim = applyDartOffset(*fitted_aim, controller_->dart_offset_rad);
    if (!final_aim || final_aim->distance_m > std::numeric_limits<float>::max()) {
        invalidate();
        return;
    }
    AimCommand command;
    command.state = AimCommand::INVALID;
    if (stability_->update(*final_aim)) {
        command.state = AimCommand::VALID;
        command.yaw_error_rad = static_cast<float>(final_aim->yaw_error_rad);
        command.distance_m = static_cast<float>(final_aim->distance_m);
    }
    command_ = command;
    target_stamp_ = target->header.stamp;
    const bool confirmed = command.state == AimCommand::VALID;
    publishVisualization(target->header.stamp,
                         p,
                         confirmed ? &*fitted_aim : nullptr,
                         confirmed ? &*final_aim : nullptr);
    tick();
}

void AimingNode::publishVisualization(const builtin_interfaces::msg::Time& stamp,
                                      const geometry_msgs::msg::Point& target,
                                      const Aim* confirmed_fitted_aim,
                                      const Aim* confirmed_final_aim) {
    if (!visualization_publisher_)
        return;

    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray markers;
    const auto makeMarker = [&](const int id, const int type) {
        Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = reference_frame_;
        marker.ns = "dart_aiming";
        marker.id = id;
        marker.type = type;
        marker.action = Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_s_);
        return marker;
    };

    auto point = makeMarker(0, Marker::SPHERE);
    point.pose.position = target;
    const double diameter = 2.0 * target_marker_radius_m_;
    point.scale.x = diameter;
    point.scale.y = diameter;
    point.scale.z = diameter;
    point.color.r = 0.1F;
    point.color.g = 1.0F;
    point.color.b = 0.1F;
    point.color.a = 1.0F;
    markers.markers.push_back(point);

    geometry_msgs::msg::Point origin;
    auto measurement = makeMarker(1, Marker::LINE_STRIP);
    measurement.scale.x = marker_line_width_m_;
    measurement.color.r = 0.1F;
    measurement.color.g = 1.0F;
    measurement.color.b = 0.1F;
    measurement.color.a = 0.8F;
    measurement.points = {origin, target};
    markers.markers.push_back(measurement);

    const auto endpointFor = [](const Aim& value) {
        geometry_msgs::msg::Point endpoint;
        endpoint.x = value.distance_m * std::cos(value.yaw_error_rad);
        endpoint.y = -value.distance_m * std::sin(value.yaw_error_rad);
        return endpoint;
    };
    const auto deleteMarker = [&](const int id, const int type) {
        auto marker = makeMarker(id, type);
        marker.action = Marker::DELETE;
        markers.markers.push_back(marker);
    };

    if (confirmed_fitted_aim && confirmed_final_aim) {
        const auto fitted_endpoint = endpointFor(*confirmed_fitted_aim);
        const auto final_endpoint = endpointFor(*confirmed_final_aim);

        // 红色是拟合结果；黄色是随后加入 dart_offset_rad 的最终输出。
        auto fitted_arrow = makeMarker(2, Marker::ARROW);
        fitted_arrow.scale.x = marker_line_width_m_;
        fitted_arrow.scale.y = 2.5 * marker_line_width_m_;
        fitted_arrow.scale.z = 3.0 * marker_line_width_m_;
        fitted_arrow.color.r = 1.0F;
        fitted_arrow.color.g = 0.1F;
        fitted_arrow.color.b = 0.1F;
        fitted_arrow.color.a = 1.0F;
        fitted_arrow.points = {origin, fitted_endpoint};
        markers.markers.push_back(fitted_arrow);

        auto fitted_point = makeMarker(3, Marker::SPHERE);
        fitted_point.pose.position = fitted_endpoint;
        const double result_diameter = 1.5 * target_marker_radius_m_;
        fitted_point.scale.x = result_diameter;
        fitted_point.scale.y = result_diameter;
        fitted_point.scale.z = result_diameter;
        fitted_point.color.r = 1.0F;
        fitted_point.color.g = 0.1F;
        fitted_point.color.b = 0.1F;
        fitted_point.color.a = 1.0F;
        markers.markers.push_back(fitted_point);

        auto final_arrow = makeMarker(5, Marker::ARROW);
        final_arrow.scale.x = marker_line_width_m_;
        final_arrow.scale.y = 2.0 * marker_line_width_m_;
        final_arrow.scale.z = 2.5 * marker_line_width_m_;
        final_arrow.color.r = 1.0F;
        final_arrow.color.g = 0.75F;
        final_arrow.color.b = 0.1F;
        final_arrow.color.a = 0.9F;
        final_arrow.points = {origin, final_endpoint};
        markers.markers.push_back(final_arrow);

        auto final_point = makeMarker(6, Marker::SPHERE);
        final_point.pose.position = final_endpoint;
        final_point.scale.x = result_diameter;
        final_point.scale.y = result_diameter;
        final_point.scale.z = result_diameter;
        final_point.color.r = 1.0F;
        final_point.color.g = 0.75F;
        final_point.color.b = 0.1F;
        final_point.color.a = 1.0F;
        markers.markers.push_back(final_point);

        const auto addAnnotation = [&](const int id,
                                       const geometry_msgs::msg::Point& endpoint,
                                       const Aim& aim,
                                       const double vertical_offset) {
            constexpr double kRadiansToDegrees = 57.29577951308232;
            auto annotation = makeMarker(id, Marker::TEXT_VIEW_FACING);
            annotation.pose.position = endpoint;
            annotation.pose.position.z += vertical_offset;
            annotation.scale.z = 1.5 * target_marker_radius_m_;
            annotation.color.r = 1.0F;
            annotation.color.g = 1.0F;
            annotation.color.b = 1.0F;
            annotation.color.a = 1.0F;
            std::ostringstream label;
            // 每组数值依次为角度（度）、距离（米）。
            label << std::fixed << std::setprecision(3)
                  << aim.yaw_error_rad * kRadiansToDegrees << '\n' << aim.distance_m;
            annotation.text = label.str();
            markers.markers.push_back(annotation);
        };
        // 红球上方显示拟合结果，黄球下方显示最终输出，避免两组文字重叠。
        addAnnotation(4, fitted_endpoint, *confirmed_fitted_aim, 3.0 * target_marker_radius_m_);
        addAnnotation(7, final_endpoint, *confirmed_final_aim, -3.0 * target_marker_radius_m_);
    } else {
        deleteMarker(2, Marker::ARROW);
        deleteMarker(3, Marker::SPHERE);
        deleteMarker(4, Marker::TEXT_VIEW_FACING);
        deleteMarker(5, Marker::ARROW);
        deleteMarker(6, Marker::SPHERE);
        deleteMarker(7, Marker::TEXT_VIEW_FACING);
    }

    visualization_publisher_->publish(markers);
    markers_visible_ = true;
}

void AimingNode::clearVisualization() {
    if (!visualization_publisher_ || !markers_visible_)
        return;
    visualization_msgs::msg::MarkerArray markers;
    for (int id = 0; id <= 7; ++id) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = now();
        marker.header.frame_id = reference_frame_;
        marker.ns = "dart_aiming";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(marker);
    }
    visualization_publisher_->publish(markers);
    markers_visible_ = false;
}

void AimingNode::tick() {
    if (!target_stamp_ || !fresh(*target_stamp_, target_timeout_s_) ||
        ((!command_ || command_->state != AimCommand::CLOSED) &&
         (!controller_ || !fresh(controller_->header.stamp, controller_timeout_s_))))
        invalidate();
    if (visualization_publisher_) {
        // TF 中的 launcher_frame 已包含串口电机角度的符号和零位转换。
        // 箭头沿镖架局部 +X，随最新 TF 转动，不重复叠加电机角度。
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp = now();
        arrow.header.frame_id = "launcher_frame";
        arrow.ns = "launcher_heading";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = controller_ && fresh(controller_->header.stamp, controller_timeout_s_)
                           ? visualization_msgs::msg::Marker::ADD
                           : visualization_msgs::msg::Marker::DELETE;
        arrow.frame_locked = true;
        arrow.pose.orientation.w = 1.0;
        arrow.scale.x = marker_line_width_m_;
        arrow.scale.y = 2.5 * marker_line_width_m_;
        arrow.scale.z = 3.0 * marker_line_width_m_;
        arrow.color.r = 0.1F;
        arrow.color.g = 0.6F;
        arrow.color.b = 1.0F;
        arrow.color.a = 1.0F;
        arrow.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_s_);
        geometry_msgs::msg::Point origin, endpoint;
        endpoint.x = 26.0;
        arrow.points = {origin, endpoint};
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(arrow);
        visualization_publisher_->publish(markers);
    }
    AimCommand message;
    message.state = AimCommand::INVALID;
    if (command_)
        message = *command_;
    message.header.stamp = now();
    message.header.frame_id = reference_frame_;
    publisher_->publish(message);
}
} // namespace dart_vision::aiming
