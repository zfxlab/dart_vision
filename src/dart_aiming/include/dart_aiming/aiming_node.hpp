#ifndef DART_AIMING_AIMING_NODE_HPP
#define DART_AIMING_AIMING_NODE_HPP

#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "dart_aiming/aiming_core.hpp"
#include "dart_interfaces/msg/aim_command.hpp"
#include "dart_interfaces/msg/controller_state.hpp"
#include "dart_interfaces/msg/stereo_target.hpp"

namespace dart_vision::aiming {

/// 接收双目位置与控制器状态，按测量时刻转换到发射架坐标系后生成瞄准指令。
class AimingNode : public rclcpp::Node {
public:
    explicit AimingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

private:
    using AimCommand = dart_interfaces::msg::AimCommand;
    using ControllerState = dart_interfaces::msg::ControllerState;
    using StereoTarget = dart_interfaces::msg::StereoTarget;

    bool fresh(const builtin_interfaces::msg::Time& stamp, double timeout) const;
    void invalidate(bool preserve_closed = false);
    void onController(ControllerState::ConstSharedPtr message);
    void onTarget(StereoTarget::ConstSharedPtr target);
    void tick();

    std::string reference_frame_;
    double target_timeout_s_{}, controller_timeout_s_{}, max_ray_gap_m_{};
    std::vector<std::int64_t> supported_target_modes_;
    std::unique_ptr<Stability> stability_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    ControllerState::ConstSharedPtr controller_;
    /// 当前缓存指令的测量时间和去重使用的最近目标时间。
    std::optional<builtin_interfaces::msg::Time> target_stamp_, last_target_stamp_;
    std::optional<AimCommand> command_;
    rclcpp::Publisher<AimCommand>::SharedPtr publisher_;
    rclcpp::Subscription<ControllerState>::SharedPtr controller_subscription_;
    rclcpp::Subscription<StereoTarget>::SharedPtr target_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace dart_vision::aiming
#endif // DART_AIMING_AIMING_NODE_HPP
