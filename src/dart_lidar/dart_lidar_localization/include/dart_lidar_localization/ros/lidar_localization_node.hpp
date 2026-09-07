#ifndef DART_LIDAR_LOCALIZATION_ROS_LIDAR_LOCALIZATION_NODE_HPP
#define DART_LIDAR_LOCALIZATION_ROS_LIDAR_LOCALIZATION_NODE_HPP

#include "dart_lidar_localization/pipeline/localization_pipeline.hpp"

#include <dart_interfaces/msg/lidar_observation.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace dart_vision::lidar::localization {

class LidarLocalizationNode : public rclcpp::Node {
public:
    explicit LidarLocalizationNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    [[nodiscard]] BaseRegistrationParameters readBaseParameters();
    [[nodiscard]] ModuleLocalizationParameters readModuleParameters();
    [[nodiscard]] Eigen::Isometry3d lookupTransform(const std::string& parent,
                                                    const std::string& child);
    [[nodiscard]] Eigen::Isometry3d initialGuessForFrame(
        const std::string& frame_id,
        const Eigen::Isometry3d& reference_from_base) const;
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
    void publishObservation(const std_msgs::msg::Header& input_header,
                            const LocalizationResult& result);
    void publishAlignedCloud(const std_msgs::msg::Header& input_header,
                             const PointCloud::ConstPtr& observation,
                             const BaseRegistrationResult& result,
                             const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
                                 publisher);
    void maybeWriteRecord(const std_msgs::msg::Header& input_header,
                          std::uint64_t measurement_id,
                          const LocalizationResult& result) const;

    std::string profile_;
    std::string input_topic_;
    std::string observation_topic_;
    std::string aligned_topic_;
    std::string candidate_aligned_topic_;
    std::string base_model_path_;
    std::string module_model_path_;
    std::string reference_frame_;
    std::string base_frame_;
    std::string rail_frame_;
    std::string initial_guess_source_;
    bool publish_aligned_cloud_{true};
    bool publish_candidate_aligned_cloud_{false};
    bool output_enabled_{false};
    double tf_lookup_timeout_s_{0.1};
    std::string output_directory_;
    std::string output_filename_prefix_;
    std::string bag_name_;
    std::uint64_t next_measurement_id_{1U};

    BaseRegistrationParameters base_parameters_;
    ModuleLocalizationParameters module_parameters_;
    PointCloud::Ptr base_model_;
    PointCloud::Ptr module_model_;
    std::unique_ptr<LocalizationPipeline> pipeline_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<dart_interfaces::msg::LidarObservation>::SharedPtr observation_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        candidate_aligned_publisher_;
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_ROS_LIDAR_LOCALIZATION_NODE_HPP
