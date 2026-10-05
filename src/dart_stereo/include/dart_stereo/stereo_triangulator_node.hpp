#ifndef DART_STEREO_STEREO_TRIANGULATOR_NODE_HPP
#define DART_STEREO_STEREO_TRIANGULATOR_NODE_HPP

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>

#include "dart_interfaces/msg/green_light_detection.hpp"
#include "dart_interfaces/msg/stereo_target.hpp"
#include "dart_stereo/stereo_triangulator.hpp"

namespace dart_vision::stereo {

/** 配对左右像素观测，根据 CameraInfo 和基线计算水平位置。 */
class StereoTriangulatorNode : public rclcpp::Node {
public:
    explicit StereoTriangulatorNode(const rclcpp::NodeOptions& options);

private:
    using GreenLightDetection = dart_interfaces::msg::GreenLightDetection;
    using StereoTarget = dart_interfaces::msg::StereoTarget;
    using CameraInfo = sensor_msgs::msg::CameraInfo;

    void observationCallback(const GreenLightDetection::ConstSharedPtr& message, bool is_left);
    void cameraInfoCallback(CameraInfo::ConstSharedPtr message, bool is_left);
    void matchQueuedObservations();
    void processPair(const GreenLightDetection& left, const GreenLightDetection& right);
    void publishFailure(const GreenLightDetection& left,
                        const GreenLightDetection& right,
                        std::uint8_t status);

    std::string left_frame_id_, right_frame_id_, reference_frame_;
    double max_pair_delta_s_{};
    std::size_t queue_size_{};
    std::unique_ptr<StereoTriangulator> triangulator_;

    std::deque<GreenLightDetection::ConstSharedPtr> left_queue_;
    std::deque<GreenLightDetection::ConstSharedPtr> right_queue_;
    std::mutex queue_mutex_;

    CameraInfo::ConstSharedPtr left_camera_info_;
    CameraInfo::ConstSharedPtr right_camera_info_;
    std::mutex camera_info_mutex_;

    rclcpp::Subscription<GreenLightDetection>::SharedPtr left_subscription_;
    rclcpp::Subscription<GreenLightDetection>::SharedPtr right_subscription_;
    rclcpp::Subscription<CameraInfo>::SharedPtr left_camera_info_subscription_;
    rclcpp::Subscription<CameraInfo>::SharedPtr right_camera_info_subscription_;
    rclcpp::Publisher<StereoTarget>::SharedPtr result_publisher_;
};

} // namespace dart_vision::stereo

#endif // DART_STEREO_STEREO_TRIANGULATOR_NODE_HPP
