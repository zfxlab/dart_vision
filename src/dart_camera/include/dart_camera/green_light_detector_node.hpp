#pragma once
#include <chrono>
#include <cstdint>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "dart_camera/green_light_detector.hpp"
#include "dart_interfaces/msg/green_light_detection.hpp"
namespace dart_vision::camera {
class GreenLightDetectorNode : public rclcpp::Node {
public:
    explicit GreenLightDetectorNode(const rclcpp::NodeOptions& options);

private:
    void declareParameters();
    GreenLightDetectorConfig readGreenLightDetectorConfig() const;
    rcl_interfaces::msg::SetParametersResult
    onParametersChanged(const std::vector<rclcpp::Parameter>&);
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

    std::string image_topic_, detection_topic_, debug_mask_topic_;
    bool publish_debug_mask_{};
    std::mutex green_light_detector_mutex_;
    GreenLightDetectorConfig green_light_detector_config_;
    std::shared_ptr<GreenLightDetector> green_light_detector_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    struct PendingImage {
        sensor_msgs::msg::Image::ConstSharedPtr image;
        std::chrono::steady_clock::time_point received;
    };
    // These queues and their timer use the default mutually exclusive callback group.
    std::deque<PendingImage> pending_images_;
    std::deque<sensor_msgs::msg::CameraInfo::ConstSharedPtr> camera_infos_;
    rclcpp::TimerBase::SharedPtr camera_info_timer_;
    rclcpp::Publisher<dart_interfaces::msg::GreenLightDetection>::SharedPtr detection_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    std::mutex diagnostic_mutex_;
    DiagnosticStatistics diagnostic_statistics_;
    std::chrono::steady_clock::time_point previous_diagnostic_time_;
};
} // namespace dart_vision::camera
