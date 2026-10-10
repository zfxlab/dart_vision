#ifndef DART_AIMING_AIMING_NODE_HPP
#define DART_AIMING_AIMING_NODE_HPP

#include <array>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cstdint>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <vector>

#include "dart_aiming/aiming_core.hpp"
#include "dart_interfaces/msg/aim_command.hpp"
#include "dart_interfaces/msg/controller_state.hpp"
#include "dart_interfaces/msg/stereo_target.hpp"

namespace dart_vision::aiming {

/// 接收双目位置与控制器状态，按测量时刻转换到输出参考系后生成瞄准指令。
class AimingNode : public rclcpp::Node {
  public:
    explicit AimingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

  private:
    using AimCommand = dart_interfaces::msg::AimCommand;
    using ControllerState = dart_interfaces::msg::ControllerState;
    using StereoTarget = dart_interfaces::msg::StereoTarget;

    enum class InvalidReason : std::size_t {
        none,
        waiting_for_target,
        stale_target,
        clock_regression,
        invalid_frame,
        invalid_target,
        controller_unavailable,
        controller_changed,
        transform_unavailable,
        geometry_invalid,
        model_invalid,
        offset_invalid,
        target_timeout,
        controller_timeout,
        count,
    };

    bool fresh(const builtin_interfaces::msg::Time& stamp, double timeout) const;
    void invalidate(bool preserve_closed = false);
    void onController(ControllerState::ConstSharedPtr message);
    void onTarget(StereoTarget::ConstSharedPtr target);
    void tick();
    void recordInvalid(InvalidReason reason);
    void recordAccepted(std::uint8_t state, const builtin_interfaces::msg::Time& stamp);
    void publishDiagnostics();

    struct DiagnosticStatistics {
        std::uint64_t targets_received_total{};
        std::uint64_t targets_received_interval{};
        std::uint64_t valid_total{};
        std::uint64_t closed_total{};
        std::uint64_t duplicate_total{};
        std::array<std::uint64_t, static_cast<std::size_t>(InvalidReason::count)>
            invalid_reason_totals{};
        std::uint8_t current_output_state{AimCommand::INVALID};
        InvalidReason last_invalid_reason{InvalidReason::waiting_for_target};
        std::deque<double> measurement_to_aim_ms;
        std::chrono::steady_clock::time_point last_target_time{};
    };

    std::string reference_frame_;
    double target_timeout_s_{}, controller_timeout_s_{}, max_height_gap_m_{};
    std::vector<std::int64_t> supported_target_modes_;
    std::unordered_map<std::uint8_t, QuadraticAimModel> fitting_models_;
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
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    std::mutex diagnostic_mutex_;
    DiagnosticStatistics diagnostic_statistics_;
    std::chrono::steady_clock::time_point previous_diagnostic_time_;
};

} // namespace dart_vision::aiming
#endif // DART_AIMING_AIMING_NODE_HPP
