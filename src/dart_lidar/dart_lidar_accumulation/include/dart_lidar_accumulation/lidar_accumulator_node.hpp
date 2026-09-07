#ifndef DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_NODE_HPP
#define DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_NODE_HPP

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <dart_interfaces/msg/controller_state.hpp>
#include <deque>
#include <mutex>
#include <optional>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vector>

#include "dart_lidar_accumulation/lidar_accumulator_config.hpp"

namespace dart_vision::lidar {

class LidarAccumulatorNode : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<PointT>;

    explicit LidarAccumulatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using SteadyTime = std::chrono::steady_clock::time_point;

    enum class MeasurementState { kWaitingForYaw, kSettling, kAccumulating, kComplete };

    struct YawSample {
        SteadyTime received_at;
        double yaw_rad{0.0};
    };

    void declareParameters();
    [[nodiscard]] LidarAccumulatorConfig readConfig() const;
    [[nodiscard]] Eigen::Vector3f readVector3(const std::string& name) const;
    [[nodiscard]] rcl_interfaces::msg::SetParametersResult
    onParametersChanged(const std::vector<rclcpp::Parameter>& parameters);
    void applyPendingConfigIfIdle();

    void
    controllerStateCallback(const dart_interfaces::msg::ControllerState::ConstSharedPtr& message);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
    void handleOnlineCloud(const sensor_msgs::msg::PointCloud2& message, SteadyTime received_at);
    void handleOfflineCloud(const sensor_msgs::msg::PointCloud2& message, SteadyTime received_at);

    void
    startMeasurement(SteadyTime stable_window_start, SteadyTime stable_declared_at, double yaw_rad);
    void abortMeasurement(const char* reason, bool clear_yaw_history);
    [[nodiscard]] bool freezeTransform(const std::string& input_frame);
    [[nodiscard]] PointCloud::Ptr
    transformAndCrop(const sensor_msgs::msg::PointCloud2& message) const;
    void addFrame(PointCloud::Ptr cloud);
    void finalizeAndPublish(SteadyTime completed_at);
    void clearAccumulation();

    mutable std::mutex config_mutex_;
    LidarAccumulatorConfig config_;
    std::optional<LidarAccumulatorConfig> pending_config_;

    MeasurementState state_{MeasurementState::kWaitingForYaw};
    std::deque<YawSample> yaw_samples_;
    bool have_controller_update_{false};
    SteadyTime last_controller_update_{};
    double frozen_yaw_rad_{0.0};

    SteadyTime measurement_origin_{};
    SteadyTime accumulation_not_before_{};
    SteadyTime measurement_deadline_{};
    SteadyTime accumulation_started_at_{};
    std::int64_t offline_window_start_ns_{0};
    std::int64_t offline_last_stamp_ns_{0};
    builtin_interfaces::msg::Time last_cloud_stamp_;

    bool frozen_transform_ready_{false};
    std::string frozen_input_frame_;
    Eigen::Isometry3d frozen_transform_{Eigen::Isometry3d::Identity()};

    std::deque<PointCloud::Ptr> frames_;
    std::size_t accumulated_points_{0U};

    std::uint64_t received_clouds_{0U};
    std::uint64_t ignored_clouds_{0U};
    std::uint64_t accepted_clouds_{0U};
    std::uint64_t dropped_tf_clouds_{0U};
    std::uint64_t dropped_empty_clouds_{0U};
    std::uint64_t aborted_measurements_{0U};
    std::uint64_t completed_measurements_{0U};

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<dart_interfaces::msg::ControllerState>::SharedPtr
        controller_state_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

} // namespace dart_vision::lidar

#endif // DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_NODE_HPP
