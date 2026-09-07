#include "dart_lidar_localization/ros/lidar_localization_node.hpp"

#include "dart_lidar_localization/io/model_loader.hpp"
#include "dart_lidar_localization/io/result_writer.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <limits>
#include <ctime>
#include <iomanip>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <sstream>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utility>
#include <vector>

namespace dart_vision::lidar::localization {
namespace {

template <typename ValueT>
void requireSameSize(const std::vector<ValueT>& values,
                     const std::size_t expected,
                     const char* name) {
    if (values.size() != expected) {
        throw std::invalid_argument(std::string(name) +
                                    " has a different number of levels");
    }
}

geometry_msgs::msg::Pose poseFromIsometry(const Eigen::Isometry3d& transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    Eigen::Quaterniond rotation(transform.linear());
    rotation.normalize();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
}

struct WallTimestamp {
    std::string iso8601;
    std::string filename;
};

WallTimestamp wallTimestampNow() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now.time_since_epoch()) %
                              1000;
    std::tm local_time{};
    localtime_r(&time, &local_time);
    WallTimestamp result;
    std::ostringstream iso;
    iso << std::put_time(&local_time, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(3)
        << std::setfill('0') << milliseconds.count() << std::put_time(&local_time, "%z");
    result.iso8601 = iso.str();
    std::ostringstream filename;
    filename << std::put_time(&local_time, "%Y%m%d_%H%M%S") << '_' << std::setw(3)
             << std::setfill('0') << milliseconds.count();
    result.filename = filename.str();
    return result;
}

} // namespace

LidarLocalizationNode::LidarLocalizationNode(const rclcpp::NodeOptions& options)
    : Node("lidar_localization_node", options),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_) {
    profile_ = declare_parameter<std::string>("profile", "online");
    input_topic_ = declare_parameter<std::string>(
        "topics.input", "lidar/debug/accumulated");
    observation_topic_ = declare_parameter<std::string>(
        "topics.observation", "lidar/observation");
    aligned_topic_ = declare_parameter<std::string>(
        "topics.aligned", "lidar/debug/aligned");
    candidate_aligned_topic_ = declare_parameter<std::string>(
        "topics.candidate_aligned", "lidar/debug/candidate_aligned");
    base_model_path_ = declare_parameter<std::string>("models.base_path", "");
    module_model_path_ = declare_parameter<std::string>("models.module_path", "");
    reference_frame_ = declare_parameter<std::string>(
        "frames.reference", "base_nominal_link");
    base_frame_ = declare_parameter<std::string>("frames.base", "base_link");
    rail_frame_ = declare_parameter<std::string>("frames.rail", "rail_origin_link");
    initial_guess_source_ = declare_parameter<std::string>(
        "initial_guess.source", "tf");
    tf_lookup_timeout_s_ = declare_parameter<double>(
        "initial_guess.lookup_timeout_s", 0.1);
    publish_aligned_cloud_ = declare_parameter<bool>("debug.publish_aligned_cloud", true);
    publish_candidate_aligned_cloud_ = declare_parameter<bool>(
        "debug.publish_candidate_aligned_cloud", false);
    output_enabled_ = declare_parameter<bool>("output.enabled", false);
    output_directory_ = declare_parameter<std::string>("output.directory", "");
    output_filename_prefix_ = declare_parameter<std::string>(
        "output.filename_prefix", "lidar_localization");
    bag_name_ = declare_parameter<std::string>("output.bag_name", "");

    base_parameters_ = readBaseParameters();
    module_parameters_ = readModuleParameters();
    if (profile_ != "online" && profile_ != "offline") {
        throw std::invalid_argument("profile must be online or offline");
    }
    if (input_topic_.empty() || observation_topic_.empty() || reference_frame_.empty() ||
        base_frame_.empty() || rail_frame_.empty()) {
        throw std::invalid_argument("localization topics and frames must not be empty");
    }
    if ((publish_aligned_cloud_ && aligned_topic_.empty()) ||
        (publish_candidate_aligned_cloud_ && candidate_aligned_topic_.empty())) {
        throw std::invalid_argument("enabled aligned-cloud topics must not be empty");
    }
    if (initial_guess_source_ != "tf" && initial_guess_source_ != "identity") {
        throw std::invalid_argument("initial_guess.source must be tf or identity");
    }
    if (!std::isfinite(tf_lookup_timeout_s_) || tf_lookup_timeout_s_ < 0.0) {
        throw std::invalid_argument("initial_guess.lookup_timeout_s must be finite and >= 0");
    }
    if (output_enabled_ && output_directory_.empty()) {
        throw std::invalid_argument(
            "output.directory is required when output.enabled is true");
    }

    base_model_ = ModelLoader::load(base_model_path_);
    if (module_parameters_.enabled) {
        module_model_ = ModelLoader::load(module_model_path_);
    }
    pipeline_ = std::make_unique<LocalizationPipeline>(base_parameters_,
                                                       module_parameters_,
                                                       base_model_,
                                                       module_model_);

    observation_publisher_ =
        create_publisher<dart_interfaces::msg::LidarObservation>(
            observation_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    if (publish_aligned_cloud_) {
        aligned_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            aligned_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }
    if (publish_candidate_aligned_cloud_) {
        candidate_aligned_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            candidate_aligned_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&LidarLocalizationNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "LiDAR localization profile=%s base_model=%s points=%zu module=%s input=%s",
                profile_.c_str(),
                ModelLoader::resolvePath(base_model_path_).c_str(),
                base_model_->size(),
                module_parameters_.enabled ? "on" : "off",
                input_topic_.c_str());
}

BaseRegistrationParameters LidarLocalizationNode::readBaseParameters() {
    BaseRegistrationParameters parameters;
    parameters.coarse_enabled = declare_parameter<bool>("base.coarse.enabled", false);

    const auto ndt_voxel = declare_parameter<std::vector<double>>(
        "base.coarse.ndt.voxel_leaf_size_m", {0.10, 0.05});
    const auto ndt_resolution = declare_parameter<std::vector<double>>(
        "base.coarse.ndt.resolution_m", {0.12, 0.06});
    const auto ndt_step = declare_parameter<std::vector<double>>(
        "base.coarse.ndt.step_size_m", {0.10, 0.05});
    const auto ndt_epsilon = declare_parameter<std::vector<double>>(
        "base.coarse.ndt.transformation_epsilon", {1.0e-3, 5.0e-4});
    const auto ndt_iterations = declare_parameter<std::vector<std::int64_t>>(
        "base.coarse.ndt.max_iterations", {60, 40});
    requireSameSize(ndt_resolution, ndt_voxel.size(), "base.coarse.ndt.resolution_m");
    requireSameSize(ndt_step, ndt_voxel.size(), "base.coarse.ndt.step_size_m");
    requireSameSize(ndt_epsilon, ndt_voxel.size(),
                    "base.coarse.ndt.transformation_epsilon");
    requireSameSize(ndt_iterations, ndt_voxel.size(), "base.coarse.ndt.max_iterations");
    for (std::size_t index = 0U; index < ndt_voxel.size(); ++index) {
        parameters.ndt_levels.push_back(NdtLevelParameters{
            ndt_voxel[index], ndt_resolution[index], ndt_step[index], ndt_epsilon[index],
            static_cast<int>(ndt_iterations[index])});
    }

    const auto gicp_voxel = declare_parameter<std::vector<double>>(
        "base.fine.gicp.voxel_leaf_size_m", {0.03, 0.02});
    const auto gicp_distance = declare_parameter<std::vector<double>>(
        "base.fine.gicp.max_correspondence_distance_m", {0.08, 0.05});
    const auto gicp_transform_epsilon = declare_parameter<std::vector<double>>(
        "base.fine.gicp.transformation_epsilon", {1.0e-5, 1.0e-5});
    const auto gicp_fitness_epsilon = declare_parameter<std::vector<double>>(
        "base.fine.gicp.fitness_epsilon", {1.0e-5, 1.0e-5});
    const auto gicp_iterations = declare_parameter<std::vector<std::int64_t>>(
        "base.fine.gicp.max_iterations", {12, 8});
    requireSameSize(gicp_distance, gicp_voxel.size(),
                    "base.fine.gicp.max_correspondence_distance_m");
    requireSameSize(gicp_transform_epsilon, gicp_voxel.size(),
                    "base.fine.gicp.transformation_epsilon");
    requireSameSize(gicp_fitness_epsilon, gicp_voxel.size(),
                    "base.fine.gicp.fitness_epsilon");
    requireSameSize(gicp_iterations, gicp_voxel.size(),
                    "base.fine.gicp.max_iterations");
    for (std::size_t index = 0U; index < gicp_voxel.size(); ++index) {
        parameters.gicp_levels.push_back(GicpLevelParameters{
            gicp_voxel[index], gicp_distance[index], gicp_transform_epsilon[index],
            gicp_fitness_epsilon[index], static_cast<int>(gicp_iterations[index])});
    }

    parameters.dof_mode = parseDofMode(
        declare_parameter<std::string>("base.constraints.dof", "xyzyaw"));
    parameters.validation.correspondence_distance_m = declare_parameter<double>(
        "base.validation.correspondence_distance_m", 0.05);
    parameters.validation.max_rmse_m = declare_parameter<double>(
        "base.validation.max_rmse_m", 0.025);
    parameters.validation.min_overlap_ratio = declare_parameter<double>(
        "base.validation.min_overlap_ratio", 0.35);
    const auto min_correspondences = declare_parameter<std::int64_t>(
        "base.validation.min_correspondences", 500);
    parameters.validation.max_translation_from_initial_m = declare_parameter<double>(
        "base.validation.max_translation_from_initial_m", 0.08);
    parameters.validation.max_yaw_from_initial_rad = declare_parameter<double>(
        "base.validation.max_yaw_from_initial_rad", 0.035);
    parameters.validation.enforce_initial_deviation_limits = declare_parameter<bool>(
        "base.validation.enforce_initial_deviation_limits", false);
    parameters.validation.reject_if_search_boundary_hit = declare_parameter<bool>(
        "base.validation.reject_if_search_boundary_hit", true);
    parameters.validation.boundary_ratio = declare_parameter<double>(
        "base.validation.boundary_ratio", 0.98);
    parameters.max_total_time_ms = declare_parameter<double>(
        "base.runtime.max_total_time_ms", 250.0);
    const auto min_input_points = declare_parameter<std::int64_t>(
        "base.runtime.min_input_points", 100);
    if (min_correspondences <= 0 || min_input_points <= 0) {
        throw std::invalid_argument("base point-count parameters must be positive");
    }
    parameters.validation.min_correspondences =
        static_cast<std::size_t>(min_correspondences);
    parameters.min_input_points = static_cast<std::size_t>(min_input_points);
    return parameters;
}

ModuleLocalizationParameters LidarLocalizationNode::readModuleParameters() {
    ModuleLocalizationParameters parameters;
    parameters.enabled = declare_parameter<bool>("module.enabled", false);
    parameters.min_position_m = declare_parameter<double>("module.min_position_m", 0.0);
    parameters.max_position_m = declare_parameter<double>("module.max_position_m", 1.2);
    parameters.coarse_step_m = declare_parameter<double>("module.coarse_step_m", 0.01);
    parameters.fine_step_m = declare_parameter<double>("module.fine_step_m", 0.001);
    parameters.fine_half_window_m = declare_parameter<double>(
        "module.fine_half_window_m", 0.02);
    parameters.max_correspondence_distance_m = declare_parameter<double>(
        "module.max_correspondence_distance_m", 0.025);
    parameters.max_rmse_m = declare_parameter<double>("module.max_rmse_m", 0.015);
    parameters.min_overlap_ratio = declare_parameter<double>(
        "module.min_overlap_ratio", 0.30);
    const auto min_correspondences = declare_parameter<std::int64_t>(
        "module.min_correspondences", 10);
    const auto roi_padding = declare_parameter<std::vector<double>>(
        "module.roi_padding_m", {0.03, 0.03, 0.03});
    parameters.max_position_jump_m = declare_parameter<double>(
        "module.max_position_jump_m", 0.0);
    if (min_correspondences <= 0 || roi_padding.size() != 3U) {
        throw std::invalid_argument(
            "module.min_correspondences must be positive and roi_padding_m must have 3 values");
    }
    parameters.min_correspondences = static_cast<std::size_t>(min_correspondences);
    parameters.roi_padding_m =
        Eigen::Vector3d(roi_padding[0], roi_padding[1], roi_padding[2]);
    return parameters;
}

Eigen::Isometry3d LidarLocalizationNode::lookupTransform(const std::string& parent,
                                                        const std::string& child) {
    const auto transform = tf_buffer_.lookupTransform(
        parent,
        child,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(tf_lookup_timeout_s_));
    return tf2::transformToEigen(transform.transform);
}

Eigen::Isometry3d LidarLocalizationNode::initialGuessForFrame(
    const std::string& frame_id,
    const Eigen::Isometry3d& reference_from_base) const {
    if (initial_guess_source_ == "identity" || frame_id == base_frame_) {
        return Eigen::Isometry3d::Identity();
    }
    if (frame_id == reference_frame_) {
        return reference_from_base.inverse();
    }
    throw std::runtime_error("Input frame " + frame_id + " matches neither " +
                             reference_frame_ + " nor " + base_frame_);
}

void LidarLocalizationNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
    const std::uint64_t measurement_id = next_measurement_id_++;
    LocalizationResult result;
    PointCloud::Ptr observation(new PointCloud);
    try {
        if (message->header.frame_id.empty()) {
            throw std::runtime_error("Accumulated point cloud has an empty frame_id");
        }
        // 对每批累积点云只冻结一次TF快照，基地初值和滑轨坐标都复用它。
        const Eigen::Isometry3d reference_from_base =
            initial_guess_source_ == "tf"
                ? lookupTransform(reference_frame_, base_frame_)
                : Eigen::Isometry3d::Identity();
        const Eigen::Isometry3d base_from_rail =
            module_parameters_.enabled
                ? lookupTransform(base_frame_, rail_frame_)
                : Eigen::Isometry3d::Identity();
        pcl::fromROSMsg(*message, *observation);
        result = pipeline_->localize(
            observation,
            initialGuessForFrame(message->header.frame_id, reference_from_base),
            reference_from_base,
            message->header.frame_id == reference_frame_,
            base_from_rail);
    } catch (const tf2::TransformException& error) {
        result.base.status = BaseRegistrationStatus::kTransformUnavailable;
        result.base.message = error.what();
        result.base.metrics.rmse_m = std::numeric_limits<double>::infinity();
        result.module.message = "required TF is unavailable";
    } catch (const std::exception& error) {
        result.base.status = BaseRegistrationStatus::kInternalError;
        result.base.message = error.what();
        result.base.metrics.rmse_m = std::numeric_limits<double>::infinity();
        result.module.message = "localization failed before module search";
    }

    publishObservation(message->header, result);
    if (result.base.hasCandidate()) {
        publishAlignedCloud(
            message->header, observation, result.base, candidate_aligned_publisher_);
    }
    if (result.base.success()) {
        publishAlignedCloud(message->header, observation, result.base, aligned_publisher_);
        RCLCPP_INFO(get_logger(),
                    "Base accepted %.1fms rmse=%.4fm overlap=%.3f; module=%s q=%.4fm",
                    result.base.elapsed_ms,
                    result.base.metrics.rmse_m,
                    result.base.metrics.overlap_ratio,
                    result.module.available ? "accepted" : "unavailable",
                    result.module.position_m);
    } else {
        RCLCPP_WARN(get_logger(), "Base unavailable: %s", result.base.message.c_str());
    }
    try {
        maybeWriteRecord(message->header, measurement_id, result);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Failed to write localization record: %s", error.what());
    }
}

void LidarLocalizationNode::publishObservation(
    const std_msgs::msg::Header& input_header,
    const LocalizationResult& result) {
    dart_interfaces::msg::LidarObservation message;
    message.header.stamp = input_header.stamp;
    message.header.frame_id = reference_frame_;
    message.base_available = result.base.success();
    message.base_pose = result.base.success()
                            ? poseFromIsometry(result.reference_from_base)
                            : poseFromIsometry(Eigen::Isometry3d::Identity());
    message.detection_module_available = result.module.available;
    message.detection_module_position_m = result.module.position_m;
    observation_publisher_->publish(message);
}

void LidarLocalizationNode::publishAlignedCloud(
    const std_msgs::msg::Header& input_header,
    const PointCloud::ConstPtr& observation,
    const BaseRegistrationResult& result,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) {
    if (!publisher || !observation) {
        return;
    }
    PointCloud aligned;
    pcl::transformPointCloud(*observation,
                             aligned,
                             result.target_from_source.matrix().cast<float>());
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(aligned, message);
    message.header.stamp = input_header.stamp;
    message.header.frame_id = base_frame_;
    publisher->publish(message);
}

void LidarLocalizationNode::maybeWriteRecord(
    const std_msgs::msg::Header& input_header,
    const std::uint64_t measurement_id,
    const LocalizationResult& result) const {
    if (!output_enabled_) {
        return;
    }
    const WallTimestamp timestamp = wallTimestampNow();
    LocalizationRecord record;
    record.generated_at = timestamp.iso8601;
    record.filename_timestamp = timestamp.filename;
    record.profile = profile_;
    record.measurement_id = measurement_id;
    record.source_stamp_sec = input_header.stamp.sec;
    record.source_stamp_nanosec = input_header.stamp.nanosec;
    record.source_frame = input_header.frame_id;
    record.reference_frame = reference_frame_;
    record.base_frame = base_frame_;
    record.rail_frame = rail_frame_;
    record.base_model_path = ModelLoader::resolvePath(base_model_path_);
    record.module_model_path = module_model_path_.empty()
                                   ? std::string{}
                                   : ModelLoader::resolvePath(module_model_path_);
    record.bag_name = bag_name_;
    record.result = result;
    record.base_parameters = base_parameters_;
    record.module_parameters = module_parameters_;
    const std::string path = ResultWriter::write(
        output_directory_, output_filename_prefix_, record);
    RCLCPP_INFO(get_logger(), "Wrote localization record to %s", path.c_str());
}

} // namespace dart_vision::lidar::localization
