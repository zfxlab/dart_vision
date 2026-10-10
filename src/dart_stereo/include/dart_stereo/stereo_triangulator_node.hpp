#ifndef DART_STEREO_STEREO_TRIANGULATOR_NODE_HPP
#define DART_STEREO_STEREO_TRIANGULATOR_NODE_HPP

#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "dart_interfaces/msg/green_light_detection.hpp"
#include "dart_interfaces/msg/stereo_target.hpp"
#include "dart_stereo/stereo_triangulator.hpp"

namespace dart_vision::stereo {

/** 配对左右单位射线，通过 TF 和三角测量计算绿灯位置。 */
class StereoTriangulatorNode : public rclcpp::Node {
  public:
    explicit StereoTriangulatorNode(const rclcpp::NodeOptions& options);

  private:
    using GreenLightDetection = dart_interfaces::msg::GreenLightDetection;
    using StereoTarget = dart_interfaces::msg::StereoTarget;
    static constexpr std::size_t kRejectionReasonCount =
        static_cast<std::size_t>(StereoTriangulationRejection::behind_reference_frame) + 1U;

    void observationCallback(const GreenLightDetection::ConstSharedPtr& message, bool is_left);
    void matchQueuedObservations();
    void processPair(const GreenLightDetection& left, const GreenLightDetection& right);
    void publishFailure(const GreenLightDetection& left, const GreenLightDetection& right,
                        std::uint8_t status);
    void recordOutput(std::uint8_t status, const rclcpp::Time& stamp);
    void publishDiagnostics();

    struct DiagnosticStatistics {
        std::uint64_t left_received_total{};
        std::uint64_t right_received_total{};
        std::uint64_t pairs_total{};
        std::uint64_t valid_total{};
        std::uint64_t closed_total{};
        std::uint64_t invalid_total{};
        std::uint64_t unmatched_left_total{};
        std::uint64_t unmatched_right_total{};
        std::uint64_t frame_mismatch_total{};
        std::uint64_t input_status_invalid_total{};
        std::uint64_t bearing_invalid_total{};
        std::uint64_t transform_errors_total{};
        std::uint64_t transform_errors_interval{};
        std::uint64_t left_received_interval{};
        std::uint64_t right_received_interval{};
        std::uint64_t pairs_interval{};
        std::uint64_t valid_interval{};
        std::array<std::uint64_t, kRejectionReasonCount> rejection_totals{};
        std::deque<double> pair_delta_ms;
        std::deque<double> measurement_to_stereo_ms;
        std::chrono::steady_clock::time_point last_output_time{};
    };

    std::string left_frame_id_, right_frame_id_, reference_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    double max_pair_delta_s_{};
    std::size_t queue_size_{};
    std::unique_ptr<StereoTriangulator> triangulator_;
    std::deque<GreenLightDetection::ConstSharedPtr> left_queue_;
    std::deque<GreenLightDetection::ConstSharedPtr> right_queue_;
    std::mutex queue_mutex_;

    rclcpp::Subscription<GreenLightDetection>::SharedPtr left_subscription_;
    rclcpp::Subscription<GreenLightDetection>::SharedPtr right_subscription_;
    rclcpp::Publisher<StereoTarget>::SharedPtr result_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    std::mutex diagnostic_mutex_;
    DiagnosticStatistics diagnostic_statistics_;
    std::chrono::steady_clock::time_point previous_diagnostic_time_;
};

} // namespace dart_vision::stereo

#endif // DART_STEREO_STEREO_TRIANGULATOR_NODE_HPP
