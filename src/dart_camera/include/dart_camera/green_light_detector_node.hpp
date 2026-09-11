#ifndef DART_CAMERA_GREEN_LIGHT_DETECTOR_NODE_HPP
#define DART_CAMERA_GREEN_LIGHT_DETECTOR_NODE_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "dart_camera/bearing_solver.hpp"
#include "dart_camera/green_light_detector.hpp"
#include "dart_interfaces/msg/camera_observation.hpp"

namespace dart_vision::camera {

/**
 * @brief 串联目标检测和像素视线计算的 ROS 2 节点。
 *
 * 节点从 image_topic 接收图像，从 camera_info_topic 获取标定参数，并向
 * observation_topic 发布相机视线观测。光源检测参数支持运行时原子更新；话题只在启动时加载。
 */
class GreenLightDetectorNode : public rclcpp::Node {
public:
    explicit GreenLightDetectorNode(const rclcpp::NodeOptions& options);

private:
    void declareParameters();

    [[nodiscard]] GreenLightDetectorConfig readGreenLightDetectorConfig() const;
    [[nodiscard]] rcl_interfaces::msg::SetParametersResult
    onParametersChanged(const std::vector<rclcpp::Parameter>& parameters);

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg);

    void publishObservation(const std_msgs::msg::Header& header,
                            const GreenLightDetectionResult& result,
                            const std::optional<cv::Vec3d>& bearing);
    void publishStatus(const std_msgs::msg::Header& header, std::uint8_t status_code);

    double last_image_received_{-1.0};
    double last_detection_stamp_{-1.0};
    std::optional<cv::Point2f> previous_center_;
    int consecutive_detections_{0};
    std::uint8_t selection_status_{0};
    rclcpp::TimerBase::SharedPtr watchdog_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string observation_topic_;
    std::string debug_mask_topic_;
    bool publish_debug_mask_{};

    mutable std::mutex green_light_detector_mutex_;
    GreenLightDetectorConfig green_light_detector_config_;
    std::shared_ptr<GreenLightDetector> green_light_detector_;
    std::shared_ptr<BearingSolver> bearing_solver_;
    std::optional<BearingSolverConfig> bearing_solver_config_;
    std::uint32_t calibration_width_{};
    std::uint32_t calibration_height_{};
    std::string calibration_frame_id_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Publisher<dart_interfaces::msg::CameraObservation>::SharedPtr observation_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_publisher_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
} // namespace dart_vision::camera

#endif // DART_CAMERA_GREEN_LIGHT_DETECTOR_NODE_HPP
