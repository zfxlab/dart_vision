#pragma once
#include <chrono>
#include <cstdint>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mutex>
#include <optional>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <unordered_map>

#include "dart_camera/green_light_detector.hpp"
#include "dart_interfaces/msg/controller_state.hpp"
#include "dart_interfaces/msg/green_light_detection.hpp"
namespace dart_vision::camera {
class GreenLightDetectorNode : public rclcpp::Node {
  public:
    explicit GreenLightDetectorNode(const rclcpp::NodeOptions& options);

  private:
    void declareParameters();
    void loadProfiles();
    GreenLightDetectorConfig readGreenLightDetectorConfig(const std::string& prefix) const;
    rcl_interfaces::msg::SetParametersResult
    onParametersChanged(const std::vector<rclcpp::Parameter>&);
    void controllerCallback(const dart_interfaces::msg::ControllerState::ConstSharedPtr&);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr&);
    void processImage(const sensor_msgs::msg::Image::ConstSharedPtr&,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr&);
    void processPendingImages();
    void publishDiagnostics();

    struct DiagnosticStatistics {
        std::uint64_t received_total{};
        std::uint64_t processed_total{};
        std::uint64_t detected_total{};
        std::uint64_t closed_total{};
        std::uint64_t no_target_total{};
        std::uint64_t errors_total{};
        std::uint64_t received_interval{};
        std::uint64_t processed_interval{};
        std::uint64_t detected_interval{};
        std::uint64_t errors_interval{};
        std::chrono::nanoseconds processing_time_interval{};
        std::chrono::nanoseconds max_processing_time_interval{};
        std::chrono::steady_clock::time_point last_processed_time{};
    };

    struct DetectorProfile {
        GreenLightDetectorConfig config;
        std::shared_ptr<GreenLightDetector> detector;
        double ambiguity_margin{};
    };

    std::string image_topic_, detection_topic_;
    std::mutex profiles_mutex_;
    std::unordered_map<std::string, DetectorProfile> profiles_;
    std::unordered_map<std::uint8_t, std::string> mode_profiles_;
    std::optional<std::string> active_profile_;
    rclcpp::Subscription<dart_interfaces::msg::ControllerState>::SharedPtr controller_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    struct PendingImage {
        sensor_msgs::msg::Image::ConstSharedPtr image;
        std::chrono::steady_clock::time_point received;
    };
    // The cache, pending queue and timer use the default mutually exclusive callback group.
    std::deque<PendingImage> pending_images_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
    rclcpp::TimerBase::SharedPtr camera_info_timer_;
    rclcpp::Publisher<dart_interfaces::msg::GreenLightDetection>::SharedPtr detection_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    std::mutex diagnostic_mutex_;
    DiagnosticStatistics diagnostic_statistics_;
    std::chrono::steady_clock::time_point previous_diagnostic_time_;
};
} // namespace dart_vision::camera
