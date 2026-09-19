#ifndef DART_STEREO_STEREO_TRIANGULATOR_NODE_HPP
#define DART_STEREO_STEREO_TRIANGULATOR_NODE_HPP

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "dart_interfaces/msg/green_light_detection.hpp"
#include "dart_interfaces/msg/stereo_target.hpp"
#include "dart_stereo/bearing_solver.hpp"
#include "dart_stereo/stereo_triangulator.hpp"

namespace dart_vision::stereo {

/** 配对左右像素检测结果，使用 CameraInfo 去畸变，并通过三角测量计算绿灯位置。 */
class StereoTriangulatorNode : public rclcpp::Node {
public:
    explicit StereoTriangulatorNode(const rclcpp::NodeOptions& options);

private:
    using GreenLightDetection = dart_interfaces::msg::GreenLightDetection;
    using StereoTarget = dart_interfaces::msg::StereoTarget;

    void observationCallback(const GreenLightDetection::ConstSharedPtr& message, bool is_left);
    void matchQueuedObservations();
    void processPair(const GreenLightDetection& left, const GreenLightDetection& right);
    void publishFailure(const GreenLightDetection& left,
                        const GreenLightDetection& right,
                        std::uint8_t status);

    std::string left_frame_id_, right_frame_id_, reference_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    using Info = sensor_msgs::msg::CameraInfo;
    std::deque<Info::ConstSharedPtr> left_infos_, right_infos_;
    rclcpp::Subscription<Info>::SharedPtr left_info_sub_, right_info_sub_;
    std::optional<cv::Vec3d> bearing(const GreenLightDetection&,
                                     const std::deque<Info::ConstSharedPtr>&);

    double max_pair_delta_s_{};
    std::size_t queue_size_{};
    std::unique_ptr<StereoTriangulator> triangulator_;
    std::deque<GreenLightDetection::ConstSharedPtr> left_queue_;
    std::deque<GreenLightDetection::ConstSharedPtr> right_queue_;
    std::mutex queue_mutex_;

    rclcpp::Subscription<GreenLightDetection>::SharedPtr left_subscription_;
    rclcpp::Subscription<GreenLightDetection>::SharedPtr right_subscription_;
    rclcpp::Publisher<StereoTarget>::SharedPtr result_publisher_;
};

} // namespace dart_vision::stereo

#endif // DART_STEREO_STEREO_TRIANGULATOR_NODE_HPP
