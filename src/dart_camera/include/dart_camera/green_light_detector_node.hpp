#pragma once
#include <mutex>
#include <rclcpp/rclcpp.hpp>
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

    std::string image_topic_, detection_topic_, debug_mask_topic_;
    bool publish_debug_mask_{};
    std::mutex green_light_detector_mutex_;
    GreenLightDetectorConfig green_light_detector_config_;
    std::shared_ptr<GreenLightDetector> green_light_detector_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<dart_interfaces::msg::GreenLightDetection>::SharedPtr detection_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
} // namespace dart_vision::camera
