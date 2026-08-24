#ifndef DART_VISION_LIDAR_LOCALIZATION_ROS_MID70_LOCALIZATION_NODE_HPP
#define DART_VISION_LIDAR_LOCALIZATION_ROS_MID70_LOCALIZATION_NODE_HPP

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dart_vision_interfaces/msg/mid70_localization.hpp"
#include "dart_vision_lidar_localization/localizers/armor_localizer.hpp"
#include "dart_vision_lidar_localization/localizers/base_localizer.hpp"
#include "dart_vision_lidar_localization/pipeline/point_cloud_accumulator.hpp"
#include "dart_vision_lidar_localization/pipeline/preprocessor.hpp"
#include "dart_vision_lidar_localization/ros/input_adapters.hpp"

namespace dart_vision::lidar {

/**
 * @brief Mid-70 point-cloud localization pipeline.
 *
 * Transform names follow t_target_source throughout. Incoming clouds are first
 * transformed with t_lidar_input into lidar_frame, which is also the
 * accumulation and registration frame. Result poses are transformed with
 * t_output_lidar before being placed in output_frame.
 */
class Mid70LocalizationNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Mid70LocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using ResultMessage = dart_vision_interfaces::msg::Mid70Localization;

    void declareParameters();
    void readParameters();
    void loadModels();
    void createRosInterfaces();

    [[nodiscard]] AlignmentMetricConfig readMetricConfig(const std::string& prefix) const;
    [[nodiscard]] Eigen::Vector3d readVector3(const std::string& name) const;
    [[nodiscard]] Eigen::Isometry3d readPose(const std::string& prefix) const;

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
    void livoxCustomCallback(const livox_interfaces::msg::CustomMsg::ConstSharedPtr& message);
    void processConvertedCloud(const CloudConversionResult& conversion,
                               const std_msgs::msg::Header& input_header) noexcept;
    void queueInputFailure(const std_msgs::msg::Header& input_header,
                           std::uint8_t status_code,
                           const std::string& status_message) noexcept;
    void processingTimerCallback() noexcept;
    void processAccumulatedCloud(const PointCloud& accumulated,
                                 const std_msgs::msg::Header& input_header,
                                 bool latest_frame_has_usable_point,
                                 std::size_t accumulated_frame_count,
                                 std::size_t accumulated_point_count) noexcept;

    [[nodiscard]] bool lookupTransform(const std::string& target_frame,
                                       const std::string& source_frame,
                                       const rclcpp::Time& stamp,
                                       Eigen::Isometry3d& t_target_source,
                                       std::string& error) const;

    [[nodiscard]] ResultMessage makeDefaultResult(const std_msgs::msg::Header& input_header,
                                                  std::size_t accumulated_frame_count,
                                                  std::size_t accumulated_point_count) const;
    void publishFailure(const std_msgs::msg::Header& input_header,
                        std::uint8_t status_code,
                        const std::string& status_message,
                        std::size_t accumulated_frame_count,
                        std::size_t accumulated_point_count,
                        bool reset_temporal = true);
    void resetTemporalState() noexcept;

    void publishCloud(
        const PointCloud& cloud,
        const std_msgs::msg::Header& header,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const;
    void publishInitialTemplateClouds();
    void publishTemplateClouds(const std_msgs::msg::Header& lidar_header,
                               const BaseLocalizationResult& base_result,
                               const ArmorLocalizationResult* armor_result,
                               const Eigen::Isometry3d& t_lidar_base_for_armor);
    void publishPose(
        const Eigen::Isometry3d& t_frame_object,
        const std_msgs::msg::Header& header,
        const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher) const;
    void publishMarkers(const std_msgs::msg::Header& lidar_header,
                        const Eigen::Isometry3d& t_lidar_base,
                        const Eigen::Isometry3d* t_lidar_armor,
                        const GreenLightEstimate* green_light,
                        bool result_valid) const;

    std::string input_type_;
    std::string pointcloud2_topic_;
    std::string livox_custom_topic_;
    std::string result_topic_;
    std::string lidar_frame_;
    std::string output_frame_;
    double tf_timeout_s_{0.05};
    double processing_rate_hz_{3.0};

    std::string base_open_pcd_path_;
    std::string base_closed_pcd_path_;
    std::string armor_pcd_path_;
    bool models_available_{false};

    std::string accumulated_topic_;
    std::string filtered_topic_;
    std::string base_open_initial_template_topic_;
    std::string base_closed_initial_template_topic_;
    std::string armor_initial_template_topic_;
    std::string base_open_template_topic_;
    std::string base_closed_template_topic_;
    std::string armor_template_topic_;
    std::string base_pose_topic_;
    std::string armor_pose_topic_;
    std::string markers_topic_;

    PointCloud::Ptr base_open_template_{new PointCloud};
    PointCloud::Ptr base_closed_template_{new PointCloud};
    PointCloud::Ptr armor_template_{new PointCloud};

    Eigen::Isometry3d t_lidar_base_initial_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d t_base_armor_zero_{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d green_offset_armor_m_{Eigen::Vector3d::Zero()};

    PointCloudAccumulatorConfig accumulator_config_{};
    PointCloudPreprocessorConfig preprocessor_config_{};
    BaseLocalizerConfig base_config_{};
    ArmorLocalizerConfig armor_config_{};

    std::unique_ptr<PointCloudAccumulator> accumulator_;
    std::unique_ptr<PointCloudPreprocessor> preprocessor_;
    std::unique_ptr<BaseLocalizer> base_localizer_;
    std::unique_ptr<ArmorLocalizer> armor_localizer_;

    std::mutex input_state_mutex_;
    std_msgs::msg::Header latest_input_header_;
    std::uint64_t latest_input_sequence_{0U};
    std::uint64_t last_processed_sequence_{0U};
    bool latest_input_is_failure_{false};
    bool latest_frame_has_usable_point_{false};
    bool temporal_reset_pending_{false};
    std::uint8_t latest_failure_status_code_{ResultMessage::STATUS_INPUT_INVALID};
    std::string latest_failure_message_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::CallbackGroup::SharedPtr input_callback_group_;
    rclcpp::CallbackGroup::SharedPtr processing_callback_group_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_subscription_;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr livox_custom_subscription_;
    rclcpp::Publisher<ResultMessage>::SharedPtr result_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        base_open_initial_template_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        base_closed_initial_template_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr armor_initial_template_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr base_open_template_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr base_closed_template_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr armor_template_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr armor_pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
};

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_LOCALIZATION_ROS_MID70_LOCALIZATION_NODE_HPP
