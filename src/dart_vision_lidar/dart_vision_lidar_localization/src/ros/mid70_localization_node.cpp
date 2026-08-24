#include "dart_vision_lidar_localization/ros/mid70_localization_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <optional>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace dart_vision::lidar {
namespace {

geometry_msgs::msg::Pose poseMessage(const Eigen::Isometry3d& t_frame_object) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = t_frame_object.translation().x();
    pose.position.y = t_frame_object.translation().y();
    pose.position.z = t_frame_object.translation().z();
    Eigen::Quaterniond quaternion(t_frame_object.linear());
    quaternion.normalize();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    return pose;
}

geometry_msgs::msg::Point pointMessage(const Eigen::Vector3d& point) {
    geometry_msgs::msg::Point message;
    message.x = point.x();
    message.y = point.y();
    message.z = point.z();
    return message;
}

void appendBoxEdges(visualization_msgs::msg::Marker& marker,
                    const Eigen::Vector3d& minimum,
                    const Eigen::Vector3d& maximum) {
    const std::array<Eigen::Vector3d, 8> corners{
        Eigen::Vector3d{minimum.x(), minimum.y(), minimum.z()},
        Eigen::Vector3d{maximum.x(), minimum.y(), minimum.z()},
        Eigen::Vector3d{maximum.x(), maximum.y(), minimum.z()},
        Eigen::Vector3d{minimum.x(), maximum.y(), minimum.z()},
        Eigen::Vector3d{minimum.x(), minimum.y(), maximum.z()},
        Eigen::Vector3d{maximum.x(), minimum.y(), maximum.z()},
        Eigen::Vector3d{maximum.x(), maximum.y(), maximum.z()},
        Eigen::Vector3d{minimum.x(), maximum.y(), maximum.z()},
    };
    constexpr std::array<std::array<std::size_t, 2>, 12> edges{{
        {{0U, 1U}},
        {{1U, 2U}},
        {{2U, 3U}},
        {{3U, 0U}},
        {{4U, 5U}},
        {{5U, 6U}},
        {{6U, 7U}},
        {{7U, 4U}},
        {{0U, 4U}},
        {{1U, 5U}},
        {{2U, 6U}},
        {{3U, 7U}},
    }};
    marker.points.reserve(edges.size() * 2U);
    for (const auto& edge : edges) {
        marker.points.push_back(pointMessage(corners[edge[0]]));
        marker.points.push_back(pointMessage(corners[edge[1]]));
    }
}

const BaseCandidate& selectedBaseCandidate(const BaseLocalizationResult& result) {
    if (result.open_candidate.valid != result.closed_candidate.valid) {
        return result.open_candidate.valid ? result.open_candidate : result.closed_candidate;
    }
    return result.open_candidate.metrics.score >= result.closed_candidate.metrics.score
               ? result.open_candidate
               : result.closed_candidate;
}

bool isTemporalStatus(const LocalizationStatus status) {
    return status == LocalizationStatus::kTemporalUnconfirmed ||
           status == LocalizationStatus::kTemporalDiscontinuity;
}

std::uint8_t resultBaseState(const BaseLocalizationResult& result) {
    if (!result.valid) {
        return dart_vision_interfaces::msg::Mid70Localization::BASE_UNKNOWN;
    }
    if (result.state == BaseState::kOpen) {
        return dart_vision_interfaces::msg::Mid70Localization::BASE_OPEN;
    }
    if (result.state == BaseState::kClosed) {
        return dart_vision_interfaces::msg::Mid70Localization::BASE_CLOSED;
    }
    return dart_vision_interfaces::msg::Mid70Localization::BASE_UNKNOWN;
}

double finiteOrNan(const double value) {
    return std::isfinite(value) ? value : std::numeric_limits<double>::quiet_NaN();
}

} // namespace

Mid70LocalizationNode::Mid70LocalizationNode(const rclcpp::NodeOptions& options)
    : Node("mid70_localization_node", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    declareParameters();
    readParameters();

    accumulator_ = std::make_unique<PointCloudAccumulator>(accumulator_config_);
    preprocessor_ = std::make_unique<PointCloudPreprocessor>(preprocessor_config_);
    loadModels();
    base_localizer_ =
        std::make_unique<BaseLocalizer>(base_open_template_, base_closed_template_, base_config_);
    armor_localizer_ =
        std::make_unique<ArmorLocalizer>(armor_template_, t_base_armor_zero_, armor_config_);
    createRosInterfaces();
    publishInitialTemplateClouds();
    std_msgs::msg::Header initial_marker_header;
    initial_marker_header.stamp = now();
    initial_marker_header.frame_id = lidar_frame_;
    publishMarkers(initial_marker_header, t_lidar_base_initial_, nullptr, nullptr, false);

    RCLCPP_INFO(get_logger(),
                "Mid-70 localization input=%s, lidar_frame=%s, output_frame=%s, "
                "processing_rate=%.2f Hz, models=%s",
                input_type_.c_str(),
                lidar_frame_.c_str(),
                output_frame_.c_str(),
                processing_rate_hz_,
                models_available_ ? "ready" : "unavailable");
}

void Mid70LocalizationNode::declareParameters() {
    declare_parameter<std::string>("input_type", "pointcloud2");
    declare_parameter<std::string>("pointcloud2_topic", "/livox/lidar");
    declare_parameter<std::string>("livox_custom_topic", "/livox/lidar");
    declare_parameter<std::string>("result_topic", "mid70_localization");
    declare_parameter<std::string>("lidar_frame", "livox_frame");
    declare_parameter<std::string>("output_frame", "livox_frame");
    declare_parameter<double>("tf_timeout_s", 0.05);
    declare_parameter<double>("processing.rate_hz", 3.0);
    declare_parameter<bool>("processing.parallel_base_candidates", true);

    declare_parameter<std::string>("model.base_open_pcd", "");
    declare_parameter<std::string>("model.base_closed_pcd", "");
    declare_parameter<std::string>("model.armor_pcd", "");

    declare_parameter<double>("accumulation.time_window_s", 0.25);
    declare_parameter<std::int64_t>("accumulation.max_frames", 5);
    declare_parameter<std::int64_t>("accumulation.max_points", 200000);
    declare_parameter<bool>("accumulation.clear_on_timestamp_regression", true);

    declare_parameter<double>("preprocess.min_distance_m", 0.3);
    declare_parameter<double>("preprocess.max_distance_m", 30.0);
    declare_parameter<bool>("preprocess.crop_box_enabled", true);
    declare_parameter<std::vector<double>>("preprocess.crop_box_min_m", {-2.0, -5.0, -2.0});
    declare_parameter<std::vector<double>>("preprocess.crop_box_max_m", {25.0, 5.0, 5.0});
    declare_parameter<bool>("preprocess.crop_box_negative", false);
    declare_parameter<bool>("preprocess.voxel_grid_enabled", true);
    declare_parameter<std::vector<double>>("preprocess.voxel_leaf_size_m", {0.02, 0.02, 0.02});
    declare_parameter<bool>("preprocess.statistical_outlier_removal_enabled", false);
    declare_parameter<std::int64_t>("preprocess.sor_mean_k", 20);
    declare_parameter<double>("preprocess.sor_stddev_mul_threshold", 1.0);

    declare_parameter<std::vector<double>>("base.initial_translation_m", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("base.initial_rpy_rad", {0.0, 0.0, 0.0});
    declare_parameter<std::int64_t>("base.icp_max_iterations", 40);
    declare_parameter<double>("base.icp_max_correspondence_distance_m", 0.15);
    declare_parameter<double>("base.icp_transformation_epsilon", 1.0e-8);
    declare_parameter<double>("base.icp_euclidean_fitness_epsilon", 1.0e-7);
    declare_parameter<double>("base.max_translation_delta_m", 0.25);
    declare_parameter<double>("base.max_rotation_delta_rad", 0.35);
    declare_parameter<bool>("base.require_icp_convergence", true);
    declare_parameter<double>("base.min_state_score_margin", 0.04);

    declare_parameter<double>("base.metrics.nearest_neighbor_truncation_m", 0.12);
    declare_parameter<double>("base.metrics.inlier_distance_m", 0.05);
    declare_parameter<double>("base.metrics.max_truncated_rmse_m", 0.08);
    declare_parameter<double>("base.metrics.min_inlier_ratio", 0.30);
    declare_parameter<double>("base.metrics.min_template_coverage_ratio", 0.03);
    declare_parameter<double>("base.metrics.min_score", 0.40);
    declare_parameter<std::int64_t>("base.metrics.min_scene_points", 50);
    declare_parameter<double>("base.metrics.residual_score_weight", 0.45);
    declare_parameter<double>("base.metrics.inlier_score_weight", 0.35);
    declare_parameter<double>("base.metrics.coverage_score_weight", 0.20);
    declare_parameter<std::int64_t>("base.temporal.required_consecutive_frames", 3);
    declare_parameter<double>("base.temporal.max_translation_jump_m", 0.08);
    declare_parameter<double>("base.temporal.max_rotation_jump_rad", 0.15);

    declare_parameter<std::vector<double>>("armor.zero_translation_m", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("armor.zero_rpy_rad", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("armor.motion_axis_base", {1.0, 0.0, 0.0});
    declare_parameter<double>("armor.min_axis_position_m", 0.0);
    declare_parameter<double>("armor.max_axis_position_m", 1.0);
    declare_parameter<double>("armor.coarse_step_m", 0.05);
    declare_parameter<double>("armor.fine_step_m", 0.005);
    declare_parameter<double>("armor.fine_half_window_m", 0.05);
    declare_parameter<std::int64_t>("armor.max_search_candidates", 10000);
    declare_parameter<double>("armor.sweep_roi_padding_m", 0.08);
    declare_parameter<double>("armor.metrics.nearest_neighbor_truncation_m", 0.10);
    declare_parameter<double>("armor.metrics.inlier_distance_m", 0.04);
    declare_parameter<double>("armor.metrics.max_truncated_rmse_m", 0.07);
    declare_parameter<double>("armor.metrics.min_inlier_ratio", 0.30);
    declare_parameter<double>("armor.metrics.min_template_coverage_ratio", 0.03);
    declare_parameter<double>("armor.metrics.min_score", 0.42);
    declare_parameter<std::int64_t>("armor.metrics.min_scene_points", 15);
    declare_parameter<double>("armor.metrics.residual_score_weight", 0.45);
    declare_parameter<double>("armor.metrics.inlier_score_weight", 0.35);
    declare_parameter<double>("armor.metrics.coverage_score_weight", 0.20);
    declare_parameter<std::int64_t>("armor.temporal.required_consecutive_frames", 3);
    declare_parameter<double>("armor.temporal.max_translation_jump_m", 0.08);
    declare_parameter<double>("armor.temporal.max_rotation_jump_rad", 0.15);

    declare_parameter<std::vector<double>>("green.offset_armor_m", {0.0, 0.0, 0.0});

    declare_parameter<std::string>("debug.accumulated_topic", "debug/accumulated");
    declare_parameter<std::string>("debug.filtered_topic", "debug/filtered");
    declare_parameter<std::string>("debug.base_open_initial_template_topic",
                                   "debug/base_open_initial_template");
    declare_parameter<std::string>("debug.base_closed_initial_template_topic",
                                   "debug/base_closed_initial_template");
    declare_parameter<std::string>("debug.armor_initial_template_topic",
                                   "debug/armor_initial_template");
    declare_parameter<std::string>("debug.base_open_template_topic", "debug/base_open_template");
    declare_parameter<std::string>("debug.base_closed_template_topic",
                                   "debug/base_closed_template");
    declare_parameter<std::string>("debug.armor_template_topic", "debug/armor_template");
    declare_parameter<std::string>("debug.base_pose_topic", "debug/base_pose");
    declare_parameter<std::string>("debug.armor_pose_topic", "debug/armor_pose");
    declare_parameter<std::string>("debug.markers_topic", "debug/markers");
}

void Mid70LocalizationNode::readParameters() {
    input_type_ = get_parameter("input_type").as_string();
    pointcloud2_topic_ = get_parameter("pointcloud2_topic").as_string();
    livox_custom_topic_ = get_parameter("livox_custom_topic").as_string();
    result_topic_ = get_parameter("result_topic").as_string();
    lidar_frame_ = get_parameter("lidar_frame").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    tf_timeout_s_ = get_parameter("tf_timeout_s").as_double();
    processing_rate_hz_ = get_parameter("processing.rate_hz").as_double();
    if ((input_type_ != "pointcloud2" && input_type_ != "livox_custom") || lidar_frame_.empty() ||
        output_frame_.empty() || !std::isfinite(tf_timeout_s_) || tf_timeout_s_ < 0.0 ||
        !std::isfinite(processing_rate_hz_) || processing_rate_hz_ <= 0.0) {
        throw std::invalid_argument("Invalid input type, frame, TF timeout, or processing rate");
    }

    base_open_pcd_path_ = get_parameter("model.base_open_pcd").as_string();
    base_closed_pcd_path_ = get_parameter("model.base_closed_pcd").as_string();
    armor_pcd_path_ = get_parameter("model.armor_pcd").as_string();

    const double window_s = get_parameter("accumulation.time_window_s").as_double();
    if (!std::isfinite(window_s) || window_s < 0.0) {
        throw std::invalid_argument("accumulation.time_window_s must be finite and non-negative");
    }
    accumulator_config_.time_window =
        std::chrono::duration_cast<CloudTimestamp>(std::chrono::duration<double>(window_s));
    const auto max_frames = get_parameter("accumulation.max_frames").as_int();
    const auto max_points = get_parameter("accumulation.max_points").as_int();
    if (max_frames <= 0 || max_points <= 0) {
        throw std::invalid_argument("accumulation max_frames and max_points must be positive");
    }
    accumulator_config_.max_frames = static_cast<std::size_t>(max_frames);
    accumulator_config_.max_points = static_cast<std::size_t>(max_points);
    accumulator_config_.timestamp_regression_policy =
        get_parameter("accumulation.clear_on_timestamp_regression").as_bool()
            ? TimestampRegressionPolicy::kClearAndAccept
            : TimestampRegressionPolicy::kRejectFrame;

    preprocessor_config_.min_distance_m = get_parameter("preprocess.min_distance_m").as_double();
    preprocessor_config_.max_distance_m = get_parameter("preprocess.max_distance_m").as_double();
    preprocessor_config_.crop_box_enabled = get_parameter("preprocess.crop_box_enabled").as_bool();
    preprocessor_config_.crop_box_min_m = readVector3("preprocess.crop_box_min_m").cast<float>();
    preprocessor_config_.crop_box_max_m = readVector3("preprocess.crop_box_max_m").cast<float>();
    preprocessor_config_.crop_box_negative =
        get_parameter("preprocess.crop_box_negative").as_bool();
    preprocessor_config_.voxel_grid_enabled =
        get_parameter("preprocess.voxel_grid_enabled").as_bool();
    preprocessor_config_.voxel_leaf_size_m =
        readVector3("preprocess.voxel_leaf_size_m").cast<float>();
    preprocessor_config_.statistical_outlier_removal_enabled =
        get_parameter("preprocess.statistical_outlier_removal_enabled").as_bool();
    preprocessor_config_.sor_mean_k =
        static_cast<int>(get_parameter("preprocess.sor_mean_k").as_int());
    preprocessor_config_.sor_stddev_mul_threshold =
        get_parameter("preprocess.sor_stddev_mul_threshold").as_double();

    t_lidar_base_initial_ = readPose("base.initial");
    base_config_.metrics = readMetricConfig("base.metrics");
    base_config_.icp_max_iterations =
        static_cast<int>(get_parameter("base.icp_max_iterations").as_int());
    base_config_.icp_max_correspondence_distance_m =
        get_parameter("base.icp_max_correspondence_distance_m").as_double();
    base_config_.icp_transformation_epsilon =
        get_parameter("base.icp_transformation_epsilon").as_double();
    base_config_.icp_euclidean_fitness_epsilon =
        get_parameter("base.icp_euclidean_fitness_epsilon").as_double();
    base_config_.max_translation_delta_m =
        get_parameter("base.max_translation_delta_m").as_double();
    base_config_.max_rotation_delta_rad = get_parameter("base.max_rotation_delta_rad").as_double();
    base_config_.require_icp_convergence = get_parameter("base.require_icp_convergence").as_bool();
    base_config_.parallel_candidates =
        get_parameter("processing.parallel_base_candidates").as_bool();
    base_config_.min_state_score_margin = get_parameter("base.min_state_score_margin").as_double();
    const auto base_required_frames =
        get_parameter("base.temporal.required_consecutive_frames").as_int();
    if (base_required_frames <= 0) {
        throw std::invalid_argument("base.temporal.required_consecutive_frames must be positive");
    }
    base_config_.temporal.required_consecutive_frames =
        static_cast<std::size_t>(base_required_frames);
    base_config_.temporal.max_translation_jump_m =
        get_parameter("base.temporal.max_translation_jump_m").as_double();
    base_config_.temporal.max_rotation_jump_rad =
        get_parameter("base.temporal.max_rotation_jump_rad").as_double();

    t_base_armor_zero_ = readPose("armor.zero");
    armor_config_.metrics = readMetricConfig("armor.metrics");
    armor_config_.motion_axis_base = readVector3("armor.motion_axis_base");
    armor_config_.min_axis_position_m = get_parameter("armor.min_axis_position_m").as_double();
    armor_config_.max_axis_position_m = get_parameter("armor.max_axis_position_m").as_double();
    armor_config_.coarse_step_m = get_parameter("armor.coarse_step_m").as_double();
    armor_config_.fine_step_m = get_parameter("armor.fine_step_m").as_double();
    armor_config_.fine_half_window_m = get_parameter("armor.fine_half_window_m").as_double();
    const auto max_search_candidates = get_parameter("armor.max_search_candidates").as_int();
    if (max_search_candidates <= 0) {
        throw std::invalid_argument("armor.max_search_candidates must be positive");
    }
    armor_config_.max_search_candidates = static_cast<std::size_t>(max_search_candidates);
    armor_config_.sweep_roi_padding_m = get_parameter("armor.sweep_roi_padding_m").as_double();
    const auto armor_required_frames =
        get_parameter("armor.temporal.required_consecutive_frames").as_int();
    if (armor_required_frames <= 0) {
        throw std::invalid_argument("armor.temporal.required_consecutive_frames must be positive");
    }
    armor_config_.temporal.required_consecutive_frames =
        static_cast<std::size_t>(armor_required_frames);
    armor_config_.temporal.max_translation_jump_m =
        get_parameter("armor.temporal.max_translation_jump_m").as_double();
    armor_config_.temporal.max_rotation_jump_rad =
        get_parameter("armor.temporal.max_rotation_jump_rad").as_double();
    green_offset_armor_m_ = readVector3("green.offset_armor_m");

    accumulated_topic_ = get_parameter("debug.accumulated_topic").as_string();
    filtered_topic_ = get_parameter("debug.filtered_topic").as_string();
    base_open_initial_template_topic_ =
        get_parameter("debug.base_open_initial_template_topic").as_string();
    base_closed_initial_template_topic_ =
        get_parameter("debug.base_closed_initial_template_topic").as_string();
    armor_initial_template_topic_ = get_parameter("debug.armor_initial_template_topic").as_string();
    base_open_template_topic_ = get_parameter("debug.base_open_template_topic").as_string();
    base_closed_template_topic_ = get_parameter("debug.base_closed_template_topic").as_string();
    armor_template_topic_ = get_parameter("debug.armor_template_topic").as_string();
    base_pose_topic_ = get_parameter("debug.base_pose_topic").as_string();
    armor_pose_topic_ = get_parameter("debug.armor_pose_topic").as_string();
    markers_topic_ = get_parameter("debug.markers_topic").as_string();
}

AlignmentMetricConfig Mid70LocalizationNode::readMetricConfig(const std::string& prefix) const {
    AlignmentMetricConfig config;
    config.nearest_neighbor_truncation_m =
        get_parameter(prefix + ".nearest_neighbor_truncation_m").as_double();
    config.inlier_distance_m = get_parameter(prefix + ".inlier_distance_m").as_double();
    config.max_truncated_rmse_m = get_parameter(prefix + ".max_truncated_rmse_m").as_double();
    config.min_inlier_ratio = get_parameter(prefix + ".min_inlier_ratio").as_double();
    config.min_template_coverage_ratio =
        get_parameter(prefix + ".min_template_coverage_ratio").as_double();
    config.min_score = get_parameter(prefix + ".min_score").as_double();
    const auto min_scene_points = get_parameter(prefix + ".min_scene_points").as_int();
    if (min_scene_points <= 0) {
        throw std::invalid_argument(prefix + ".min_scene_points must be positive");
    }
    config.min_scene_points = static_cast<std::size_t>(min_scene_points);
    config.residual_score_weight = get_parameter(prefix + ".residual_score_weight").as_double();
    config.inlier_score_weight = get_parameter(prefix + ".inlier_score_weight").as_double();
    config.coverage_score_weight = get_parameter(prefix + ".coverage_score_weight").as_double();
    return config;
}

Eigen::Vector3d Mid70LocalizationNode::readVector3(const std::string& name) const {
    const std::vector<double> values = get_parameter(name).as_double_array();
    if (values.size() != 3U || !std::all_of(values.begin(), values.end(), [](const double value) {
            return std::isfinite(value);
        })) {
        throw std::invalid_argument(name + " must contain exactly three finite numbers");
    }
    return {values[0], values[1], values[2]};
}

Eigen::Isometry3d Mid70LocalizationNode::readPose(const std::string& prefix) const {
    const Eigen::Vector3d translation = readVector3(prefix + "_translation_m");
    const Eigen::Vector3d rpy = readVector3(prefix + "_rpy_rad");
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
                             .toRotationMatrix();
    transform.translation() = translation;
    return transform;
}

void Mid70LocalizationNode::loadModels() {
    const auto load = [this](const std::string& path,
                             const char* label,
                             const PointCloud::Ptr& cloud) {
        cloud->clear();
        if (path.empty()) {
            RCLCPP_ERROR(get_logger(), "%s model path is empty", label);
            return false;
        }
        pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, xyz_cloud) < 0 || xyz_cloud.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to load non-empty %s PCD: %s", label, path.c_str());
            return false;
        }
        cloud->reserve(xyz_cloud.size());
        for (const auto& source : xyz_cloud) {
            if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) {
                continue;
            }
            PointT point;
            point.x = source.x;
            point.y = source.y;
            point.z = source.z;
            point.intensity = 0.0F;
            cloud->push_back(point);
        }
        if (cloud->empty()) {
            RCLCPP_ERROR(
                get_logger(), "%s PCD contains no finite XYZ points: %s", label, path.c_str());
            return false;
        }
        finalizePointCloudMetadata(*cloud);
        RCLCPP_INFO(get_logger(),
                    "Loaded %s template: %zu points from %s",
                    label,
                    cloud->size(),
                    path.c_str());
        return true;
    };

    const bool open_ready = load(base_open_pcd_path_, "base-open-fixed", base_open_template_);
    const bool closed_ready =
        load(base_closed_pcd_path_, "base-closed-fixed", base_closed_template_);
    const bool armor_ready = load(armor_pcd_path_, "moving-armor", armor_template_);
    models_available_ = open_ready && closed_ready && armor_ready;
}

void Mid70LocalizationNode::createRosInterfaces() {
    input_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    processing_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    const auto debug_cloud_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    const auto initial_template_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    result_publisher_ = create_publisher<ResultMessage>(result_topic_, rclcpp::QoS(10));
    accumulated_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(accumulated_topic_, debug_cloud_qos);
    filtered_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic_, debug_cloud_qos);
    base_open_initial_template_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        base_open_initial_template_topic_, initial_template_qos);
    base_closed_initial_template_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        base_closed_initial_template_topic_, initial_template_qos);
    armor_initial_template_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        armor_initial_template_topic_, initial_template_qos);
    base_open_template_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(base_open_template_topic_, debug_cloud_qos);
    base_closed_template_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        base_closed_template_topic_, debug_cloud_qos);
    armor_template_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(armor_template_topic_, debug_cloud_qos);
    base_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(base_pose_topic_, rclcpp::QoS(10));
    armor_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(armor_pose_topic_, rclcpp::QoS(10));
    markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        markers_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = input_callback_group_;
    if (input_type_ == "pointcloud2") {
        pointcloud2_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud2_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&Mid70LocalizationNode::pointCloud2Callback, this, std::placeholders::_1),
            subscription_options);
    } else {
        livox_custom_subscription_ = create_subscription<livox_interfaces::msg::CustomMsg>(
            livox_custom_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&Mid70LocalizationNode::livoxCustomCallback, this, std::placeholders::_1),
            subscription_options);
    }

    const auto processing_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / processing_rate_hz_));
    processing_timer_ = create_wall_timer(
        processing_period, [this]() { processingTimerCallback(); }, processing_callback_group_);
}

void Mid70LocalizationNode::pointCloud2Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
    try {
        const CloudConversionResult conversion = convertPointCloud2(*message);
        processConvertedCloud(conversion, message->header);
    } catch (const std::exception& error) {
        queueInputFailure(message->header,
                          ResultMessage::STATUS_INPUT_INVALID,
                          std::string("PointCloud2 conversion exception: ") + error.what());
    }
}

void Mid70LocalizationNode::livoxCustomCallback(
    const livox_interfaces::msg::CustomMsg::ConstSharedPtr& message) {
    try {
        const CloudConversionResult conversion = convertLivoxCustomMsg(*message);
        processConvertedCloud(conversion, message->header);
    } catch (const std::exception& error) {
        queueInputFailure(message->header,
                          ResultMessage::STATUS_INPUT_INVALID,
                          std::string("Livox CustomMsg conversion exception: ") + error.what());
    }
}

void Mid70LocalizationNode::processConvertedCloud(
    const CloudConversionResult& conversion, const std_msgs::msg::Header& input_header) noexcept {
    try {
        if (!conversion.success) {
            queueInputFailure(input_header,
                              ResultMessage::STATUS_INPUT_INVALID,
                              "input_conversion_failed: " + conversion.error);
            return;
        }
        if (!conversion.warning.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "%s", conversion.warning.c_str());
        }
        if (input_header.frame_id.empty()) {
            queueInputFailure(input_header,
                              ResultMessage::STATUS_INPUT_INVALID,
                              "input header.frame_id is empty");
            return;
        }

        const rclcpp::Time stamp(input_header.stamp);
        Eigen::Isometry3d t_lidar_input = Eigen::Isometry3d::Identity();
        std::string transform_error;
        if (!lookupTransform(
                lidar_frame_, input_header.frame_id, stamp, t_lidar_input, transform_error)) {
            queueInputFailure(input_header,
                              ResultMessage::STATUS_TF_UNAVAILABLE,
                              "t_lidar_input unavailable: " + transform_error);
            return;
        }

        PointCloud cloud_lidar = transformPointCloud(conversion.cloud, t_lidar_input, lidar_frame_);
        const bool current_frame_has_usable_point = preprocessor_->hasUsablePoint(cloud_lidar);
        AddFrameResult add_result;
        {
            std::lock_guard<std::mutex> lock(input_state_mutex_);
            add_result = accumulator_->addFrame(cloud_lidar, CloudTimestamp(stamp.nanoseconds()));
            if (add_result.accepted()) {
                latest_input_header_ = input_header;
                latest_input_is_failure_ = false;
                latest_frame_has_usable_point_ =
                    add_result.stored_input_points > 0U && current_frame_has_usable_point;
                ++latest_input_sequence_;
                if (add_result.status == AddFrameStatus::kAcceptedAfterTimeReset) {
                    temporal_reset_pending_ = true;
                }
            }
        }
        if (!add_result.accepted()) {
            queueInputFailure(input_header,
                              ResultMessage::STATUS_INPUT_INVALID,
                              "input timestamp regressed and frame was rejected");
            return;
        }
        if (add_result.status == AddFrameStatus::kAcceptedAfterTimeReset) {
            RCLCPP_WARN(get_logger(),
                        "Input timestamp regressed; accumulation and temporal state reset");
        }
    } catch (const std::exception& error) {
        queueInputFailure(input_header,
                          ResultMessage::STATUS_INPUT_INVALID,
                          std::string("input front-end exception: ") + error.what());
    }
}

void Mid70LocalizationNode::queueInputFailure(const std_msgs::msg::Header& input_header,
                                              const std::uint8_t status_code,
                                              const std::string& status_message) noexcept {
    try {
        std::lock_guard<std::mutex> lock(input_state_mutex_);
        latest_input_header_ = input_header;
        latest_input_is_failure_ = true;
        latest_failure_status_code_ = status_code;
        latest_failure_message_ = status_message;
        temporal_reset_pending_ = true;
        ++latest_input_sequence_;
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Failed to queue input error: %s", error.what());
    }
}

void Mid70LocalizationNode::processingTimerCallback() noexcept {
    PointCloud accumulated;
    std_msgs::msg::Header input_header;
    std::uint8_t failure_status_code = ResultMessage::STATUS_INPUT_INVALID;
    std::string failure_message;
    std::size_t accumulated_frame_count = 0U;
    std::size_t accumulated_point_count = 0U;
    bool input_is_failure = false;
    bool latest_frame_has_usable_point = false;
    bool reset_temporal = false;

    try {
        {
            std::lock_guard<std::mutex> lock(input_state_mutex_);
            if (latest_input_sequence_ == 0U ||
                latest_input_sequence_ == last_processed_sequence_) {
                return;
            }
            last_processed_sequence_ = latest_input_sequence_;
            input_header = latest_input_header_;
            input_is_failure = latest_input_is_failure_;
            latest_frame_has_usable_point = latest_frame_has_usable_point_;
            failure_status_code = latest_failure_status_code_;
            failure_message = latest_failure_message_;
            reset_temporal = temporal_reset_pending_;
            temporal_reset_pending_ = false;
            accumulated_frame_count = accumulator_->frameCount();
            accumulated_point_count = accumulator_->pointCount();
            if (!input_is_failure) {
                accumulated = accumulator_->accumulatedCloud();
            }
        }

        if (reset_temporal) {
            resetTemporalState();
        }
        if (input_is_failure) {
            publishFailure(input_header,
                           failure_status_code,
                           failure_message,
                           accumulated_frame_count,
                           accumulated_point_count,
                           false);
            return;
        }
        processAccumulatedCloud(accumulated,
                                input_header,
                                latest_frame_has_usable_point,
                                accumulated_frame_count,
                                accumulated_point_count);
    } catch (const std::exception& error) {
        publishFailure(input_header,
                       ResultMessage::STATUS_INPUT_INVALID,
                       std::string("processing snapshot exception: ") + error.what(),
                       accumulated_frame_count,
                       accumulated_point_count);
    }
}

void Mid70LocalizationNode::processAccumulatedCloud(
    const PointCloud& accumulated,
    const std_msgs::msg::Header& input_header,
    const bool latest_frame_has_usable_point,
    const std::size_t accumulated_frame_count,
    const std::size_t accumulated_point_count) noexcept {
    const auto processing_started = std::chrono::steady_clock::now();
    try {
        const rclcpp::Time stamp(input_header.stamp);
        std_msgs::msg::Header lidar_header = input_header;
        lidar_header.frame_id = lidar_frame_;
        publishCloud(accumulated, lidar_header, accumulated_publisher_);
        if (accumulated.empty() || !latest_frame_has_usable_point) {
            publishFailure(input_header,
                           ResultMessage::STATUS_INSUFFICIENT_POINTS,
                           "input cloud contains no finite point inside the configured range/ROI",
                           accumulated_frame_count,
                           accumulated_point_count);
            return;
        }

        PointCloudPreprocessingStats preprocessing_stats;
        const PointCloud filtered = preprocessor_->process(accumulated, &preprocessing_stats);
        publishCloud(filtered, lidar_header, filtered_publisher_);

        if (!models_available_) {
            publishFailure(input_header,
                           ResultMessage::STATUS_MODEL_UNAVAILABLE,
                           "one or more independent template PCD files are unavailable",
                           accumulated_frame_count,
                           accumulated_point_count);
            return;
        }
        if (filtered.size() < base_config_.metrics.min_scene_points) {
            publishFailure(input_header,
                           ResultMessage::STATUS_INSUFFICIENT_POINTS,
                           "filtered cloud has " + std::to_string(filtered.size()) +
                               " points; base requires at least " +
                               std::to_string(base_config_.metrics.min_scene_points),
                           accumulated_frame_count,
                           accumulated_point_count);
            return;
        }

        Eigen::Isometry3d t_output_lidar = Eigen::Isometry3d::Identity();
        std::string transform_error;
        const bool output_transform_available =
            lookupTransform(output_frame_, lidar_frame_, stamp, t_output_lidar, transform_error);

        const BaseLocalizationResult base_result =
            base_localizer_->localize(filtered, t_lidar_base_initial_);
        std::optional<ArmorLocalizationResult> armor_result;
        if (base_result.valid) {
            armor_result = armor_localizer_->localize(filtered, base_result.t_registration_base);
        } else {
            armor_localizer_->reset();
        }

        publishTemplateClouds(lidar_header,
                              base_result,
                              armor_result ? &*armor_result : nullptr,
                              base_result.t_registration_base);

        ResultMessage result =
            makeDefaultResult(input_header, accumulated_frame_count, accumulated_point_count);
        const BaseCandidate& base_candidate = selectedBaseCandidate(base_result);
        result.base_valid = base_result.valid;
        result.base_state = resultBaseState(base_result);
        result.base_confidence = static_cast<float>(base_result.confidence);
        result.base_state_score_margin = static_cast<float>(base_result.state_score_margin);
        result.base_consecutive_frames = static_cast<std::uint32_t>(std::min<std::size_t>(
            base_result.consecutive_frames, std::numeric_limits<std::uint32_t>::max()));
        result.base_truncated_residual_m =
            static_cast<float>(finiteOrNan(base_candidate.metrics.truncated_rmse_m));
        result.base_inlier_rate = static_cast<float>(base_candidate.metrics.inlier_ratio);
        result.base_coverage = static_cast<float>(base_candidate.metrics.template_coverage_ratio);

        if (isFiniteTransform(base_result.t_registration_base)) {
            const Eigen::Isometry3d t_pose_base =
                output_transform_available ? t_output_lidar * base_result.t_registration_base
                                           : base_result.t_registration_base;
            if (!output_transform_available) {
                result.base_pose.header = lidar_header;
            }
            result.base_pose.pose = poseMessage(t_pose_base);
            publishPose(t_pose_base, result.base_pose.header, base_pose_publisher_);
        }

        if (armor_result) {
            result.armor_valid = armor_result->valid;
            result.armor_confidence = static_cast<float>(armor_result->confidence);
            result.armor_consecutive_frames = static_cast<std::uint32_t>(std::min<std::size_t>(
                armor_result->consecutive_frames, std::numeric_limits<std::uint32_t>::max()));
            result.armor_axis_position_m = armor_result->axis_position_m;
            result.armor_truncated_residual_m = static_cast<float>(
                finiteOrNan(armor_result->best_candidate.metrics.truncated_rmse_m));
            result.armor_inlier_rate =
                static_cast<float>(armor_result->best_candidate.metrics.inlier_ratio);
            result.armor_coverage =
                static_cast<float>(armor_result->best_candidate.metrics.template_coverage_ratio);
            if (armor_result->evaluated_candidates > 0U &&
                isFiniteTransform(armor_result->t_registration_armor)) {
                const Eigen::Isometry3d t_pose_armor =
                    output_transform_available ? t_output_lidar * armor_result->t_registration_armor
                                               : armor_result->t_registration_armor;
                if (!output_transform_available) {
                    result.armor_pose.header = lidar_header;
                }
                result.armor_pose.pose = poseMessage(t_pose_armor);
                publishPose(t_pose_armor, result.armor_pose.header, armor_pose_publisher_);
            }
        }

        std::optional<GreenLightEstimate> green_light;
        if (armor_result && armor_result->valid) {
            green_light = computeGreenLightEstimate(
                armor_result->t_registration_armor,
                green_offset_armor_m_,
                output_transform_available ? t_output_lidar : Eigen::Isometry3d::Identity());
            if (green_light->valid) {
                result.green_light_lidar.point = pointMessage(green_light->position_registration_m);
                result.green_light_distance_m = green_light->distance_from_registration_origin_m;
                if (output_transform_available) {
                    result.green_light_output.point = pointMessage(green_light->position_output_m);
                }
            }
        }

        result.confidence =
            armor_result
                ? static_cast<float>(std::min(base_result.confidence, armor_result->confidence))
                : static_cast<float>(base_result.confidence);
        if (!output_transform_available) {
            result.status_code = ResultMessage::STATUS_OUTPUT_TF_UNAVAILABLE;
            result.status_message = "t_output_lidar unavailable: " + transform_error;
        } else if (!base_result.valid) {
            result.status_code = isTemporalStatus(base_result.status)
                                     ? ResultMessage::STATUS_BASE_UNSTABLE
                                     : ResultMessage::STATUS_BASE_UNRELIABLE;
            result.status_message = "base: " + std::string(toString(base_result.status));
        } else if (!armor_result || !armor_result->valid) {
            const LocalizationStatus armor_status =
                armor_result ? armor_result->status : LocalizationStatus::kNoSearchCandidate;
            result.status_code = isTemporalStatus(armor_status)
                                     ? ResultMessage::STATUS_ARMOR_UNSTABLE
                                     : ResultMessage::STATUS_ARMOR_UNRELIABLE;
            result.status_message = "armor: " + std::string(toString(armor_status));
        } else if (!green_light || !green_light->valid) {
            result.status_code = ResultMessage::STATUS_ARMOR_UNRELIABLE;
            result.status_message = "green-light coordinate transform is non-finite";
        } else {
            result.valid = true;
            result.status_code = ResultMessage::STATUS_OK;
            result.status_message = "valid";
        }

        const Eigen::Isometry3d* marker_armor_pose =
            armor_result && armor_result->evaluated_candidates > 0U
                ? &armor_result->t_registration_armor
                : nullptr;
        publishMarkers(lidar_header,
                       base_result.t_registration_base,
                       marker_armor_pose,
                       green_light ? &*green_light : nullptr,
                       result.valid);
        result_publisher_->publish(result);

        const auto elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - processing_started);
        RCLCPP_INFO_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Localization backend %.1f ms, snapshot=%zu frames/%zu points, "
                             "filtered=%zu points",
                             elapsed.count(),
                             accumulated_frame_count,
                             accumulated_point_count,
                             filtered.size());
    } catch (const std::exception& error) {
        publishFailure(input_header,
                       ResultMessage::STATUS_INPUT_INVALID,
                       std::string("processing exception: ") + error.what(),
                       accumulated_frame_count,
                       accumulated_point_count);
    }
}

bool Mid70LocalizationNode::lookupTransform(const std::string& target_frame,
                                            const std::string& source_frame,
                                            const rclcpp::Time& stamp,
                                            Eigen::Isometry3d& t_target_source,
                                            std::string& error) const {
    if (target_frame == source_frame) {
        t_target_source.setIdentity();
        error.clear();
        return true;
    }
    try {
        const auto transform = tf_buffer_.lookupTransform(
            target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(tf_timeout_s_));
        t_target_source = tf2::transformToEigen(transform);
        if (!isFiniteTransform(t_target_source)) {
            error = "transform contains non-finite values";
            return false;
        }
        error.clear();
        return true;
    } catch (const tf2::TransformException& exception) {
        error = exception.what();
        return false;
    }
}

Mid70LocalizationNode::ResultMessage
Mid70LocalizationNode::makeDefaultResult(const std_msgs::msg::Header& input_header,
                                         const std::size_t accumulated_frame_count,
                                         const std::size_t accumulated_point_count) const {
    ResultMessage result;
    result.header = input_header;
    result.header.frame_id = output_frame_;
    result.valid = false;
    result.confidence = 0.0F;
    result.status_code = ResultMessage::STATUS_INPUT_INVALID;
    result.status_message = "not_evaluated";
    result.base_valid = false;
    result.base_state = ResultMessage::BASE_UNKNOWN;
    result.armor_valid = false;

    result.base_pose.header = result.header;
    result.armor_pose.header = result.header;
    result.base_pose.pose.orientation.w = 1.0;
    result.armor_pose.pose.orientation.w = 1.0;
    result.green_light_lidar.header = input_header;
    result.green_light_lidar.header.frame_id = lidar_frame_;
    result.green_light_output.header = result.header;

    const double nan = std::numeric_limits<double>::quiet_NaN();
    result.base_truncated_residual_m = static_cast<float>(nan);
    result.armor_axis_position_m = nan;
    result.armor_truncated_residual_m = static_cast<float>(nan);
    result.green_light_lidar.point.x = nan;
    result.green_light_lidar.point.y = nan;
    result.green_light_lidar.point.z = nan;
    result.green_light_output.point.x = nan;
    result.green_light_output.point.y = nan;
    result.green_light_output.point.z = nan;
    result.green_light_distance_m = nan;
    result.accumulated_frame_count = static_cast<std::uint32_t>(
        std::min<std::size_t>(accumulated_frame_count, std::numeric_limits<std::uint32_t>::max()));
    result.accumulated_point_count = static_cast<std::uint32_t>(
        std::min<std::size_t>(accumulated_point_count, std::numeric_limits<std::uint32_t>::max()));
    return result;
}

void Mid70LocalizationNode::publishFailure(const std_msgs::msg::Header& input_header,
                                           const std::uint8_t status_code,
                                           const std::string& status_message,
                                           const std::size_t accumulated_frame_count,
                                           const std::size_t accumulated_point_count,
                                           const bool reset_temporal) {
    if (reset_temporal) {
        resetTemporalState();
    }
    ResultMessage result =
        makeDefaultResult(input_header, accumulated_frame_count, accumulated_point_count);
    result.status_code = status_code;
    result.status_message = status_message;
    result_publisher_->publish(result);
    std_msgs::msg::Header lidar_header = input_header;
    lidar_header.frame_id = lidar_frame_;
    const PointCloud empty_cloud;
    publishCloud(empty_cloud, lidar_header, base_open_template_publisher_);
    publishCloud(empty_cloud, lidar_header, base_closed_template_publisher_);
    publishCloud(empty_cloud, lidar_header, armor_template_publisher_);
    publishMarkers(lidar_header, t_lidar_base_initial_, nullptr, nullptr, false);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", status_message.c_str());
}

void Mid70LocalizationNode::resetTemporalState() noexcept {
    if (base_localizer_) {
        base_localizer_->reset();
    }
    if (armor_localizer_) {
        armor_localizer_->reset();
    }
}

void Mid70LocalizationNode::publishCloud(
    const PointCloud& cloud,
    const std_msgs::msg::Header& header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const {
    if (!publisher) {
        return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    publisher->publish(message);
}

void Mid70LocalizationNode::publishInitialTemplateClouds() {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = lidar_frame_;
    if (!base_open_template_->empty()) {
        const PointCloud transformed =
            transformPointCloud(*base_open_template_, t_lidar_base_initial_, lidar_frame_);
        publishCloud(transformed, header, base_open_initial_template_publisher_);
    }
    if (!base_closed_template_->empty()) {
        const PointCloud transformed =
            transformPointCloud(*base_closed_template_, t_lidar_base_initial_, lidar_frame_);
        publishCloud(transformed, header, base_closed_initial_template_publisher_);
    }
    if (!armor_template_->empty()) {
        const Eigen::Isometry3d t_lidar_armor_initial = t_lidar_base_initial_ * t_base_armor_zero_;
        const PointCloud transformed =
            transformPointCloud(*armor_template_, t_lidar_armor_initial, lidar_frame_);
        publishCloud(transformed, header, armor_initial_template_publisher_);
    }
}

void Mid70LocalizationNode::publishTemplateClouds(const std_msgs::msg::Header& lidar_header,
                                                  const BaseLocalizationResult& base_result,
                                                  const ArmorLocalizationResult* armor_result,
                                                  const Eigen::Isometry3d& t_lidar_base_for_armor) {
    const Eigen::Isometry3d t_lidar_open =
        isFiniteTransform(base_result.open_candidate.t_registration_base)
            ? base_result.open_candidate.t_registration_base
            : t_lidar_base_initial_;
    const Eigen::Isometry3d t_lidar_closed =
        isFiniteTransform(base_result.closed_candidate.t_registration_base)
            ? base_result.closed_candidate.t_registration_base
            : t_lidar_base_initial_;
    if (!base_open_template_->empty()) {
        const PointCloud transformed =
            transformPointCloud(*base_open_template_, t_lidar_open, lidar_frame_);
        publishCloud(transformed, lidar_header, base_open_template_publisher_);
    }
    if (!base_closed_template_->empty()) {
        const PointCloud transformed =
            transformPointCloud(*base_closed_template_, t_lidar_closed, lidar_frame_);
        publishCloud(transformed, lidar_header, base_closed_template_publisher_);
    }
    if (!armor_template_->empty()) {
        Eigen::Isometry3d t_lidar_armor = t_lidar_base_for_armor * t_base_armor_zero_;
        if (armor_result && armor_result->evaluated_candidates > 0U &&
            isFiniteTransform(armor_result->t_registration_armor)) {
            t_lidar_armor = armor_result->t_registration_armor;
        }
        const PointCloud transformed =
            transformPointCloud(*armor_template_, t_lidar_armor, lidar_frame_);
        publishCloud(transformed, lidar_header, armor_template_publisher_);
    }
}

void Mid70LocalizationNode::publishPose(
    const Eigen::Isometry3d& t_frame_object,
    const std_msgs::msg::Header& header,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher) const {
    if (!publisher || !isFiniteTransform(t_frame_object)) {
        return;
    }
    geometry_msgs::msg::PoseStamped message;
    message.header = header;
    message.pose = poseMessage(t_frame_object);
    publisher->publish(message);
}

void Mid70LocalizationNode::publishMarkers(const std_msgs::msg::Header& lidar_header,
                                           const Eigen::Isometry3d& t_lidar_base,
                                           const Eigen::Isometry3d* t_lidar_armor,
                                           const GreenLightEstimate* green_light,
                                           const bool result_valid) const {
    if (!markers_publisher_) {
        return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker axis;
    axis.header = lidar_header;
    axis.ns = "mid70_motion_axis";
    axis.id = 0;
    axis.type = visualization_msgs::msg::Marker::LINE_LIST;
    axis.action = visualization_msgs::msg::Marker::ADD;
    axis.pose.orientation.w = 1.0;
    axis.scale.x = 0.02;
    axis.color.r = 0.1F;
    axis.color.g = 0.6F;
    axis.color.b = 1.0F;
    axis.color.a = 1.0F;
    if (isFiniteTransform(t_lidar_base) && armor_config_.motion_axis_base.norm() > 1.0e-12) {
        const Eigen::Vector3d axis_base = armor_config_.motion_axis_base.normalized();
        const Eigen::Vector3d zero_origin_base = t_base_armor_zero_.translation();
        const Eigen::Vector3d minimum_lidar =
            t_lidar_base * (zero_origin_base + axis_base * armor_config_.min_axis_position_m);
        const Eigen::Vector3d maximum_lidar =
            t_lidar_base * (zero_origin_base + axis_base * armor_config_.max_axis_position_m);
        axis.points.push_back(pointMessage(minimum_lidar));
        axis.points.push_back(pointMessage(maximum_lidar));
    }
    markers.markers.push_back(axis);

    visualization_msgs::msg::Marker green;
    green.header = lidar_header;
    green.ns = "mid70_green_light";
    green.id = 1;
    green.type = visualization_msgs::msg::Marker::SPHERE;
    green.scale.x = 0.08;
    green.scale.y = 0.08;
    green.scale.z = 0.08;
    green.color.r = result_valid ? 0.0F : 1.0F;
    green.color.g = result_valid ? 1.0F : 0.1F;
    green.color.b = 0.1F;
    green.color.a = 1.0F;
    if (green_light && green_light->valid) {
        green.action = visualization_msgs::msg::Marker::ADD;
        green.pose.position = pointMessage(green_light->position_registration_m);
        green.pose.orientation.w = 1.0;
    } else {
        green.action = visualization_msgs::msg::Marker::DELETE;
    }
    markers.markers.push_back(green);

    visualization_msgs::msg::Marker armor;
    armor.header = lidar_header;
    armor.ns = "mid70_armor_pose";
    armor.id = 2;
    armor.type = visualization_msgs::msg::Marker::ARROW;
    armor.scale.x = 0.25;
    armor.scale.y = 0.04;
    armor.scale.z = 0.04;
    armor.color.r = 1.0F;
    armor.color.g = 0.65F;
    armor.color.b = 0.0F;
    armor.color.a = 1.0F;
    if (t_lidar_armor && isFiniteTransform(*t_lidar_armor)) {
        armor.action = visualization_msgs::msg::Marker::ADD;
        armor.pose = poseMessage(*t_lidar_armor);
    } else {
        armor.action = visualization_msgs::msg::Marker::DELETE;
    }
    markers.markers.push_back(armor);

    visualization_msgs::msg::Marker status;
    status.header = lidar_header;
    status.ns = "mid70_status";
    status.id = 3;
    status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    status.action = visualization_msgs::msg::Marker::ADD;
    status.pose.orientation.w = 1.0;
    if (isFiniteTransform(t_lidar_base)) {
        status.pose.position =
            pointMessage(t_lidar_base.translation() + Eigen::Vector3d(0, 0, 0.5));
    }
    status.scale.z = 0.12;
    status.color.r = result_valid ? 0.0F : 1.0F;
    status.color.g = result_valid ? 1.0F : 0.0F;
    status.color.b = 0.0F;
    status.color.a = 1.0F;
    status.text = result_valid ? "Mid70 VALID" : "Mid70 INVALID";
    markers.markers.push_back(status);

    visualization_msgs::msg::Marker crop_box;
    crop_box.header = lidar_header;
    crop_box.ns = "mid70_crop_box";
    crop_box.id = 4;
    crop_box.type = visualization_msgs::msg::Marker::LINE_LIST;
    crop_box.pose.orientation.w = 1.0;
    crop_box.scale.x = 0.025;
    crop_box.color.r = 0.0F;
    crop_box.color.g = 1.0F;
    crop_box.color.b = 1.0F;
    crop_box.color.a = 0.9F;
    if (preprocessor_config_.crop_box_enabled) {
        crop_box.action = visualization_msgs::msg::Marker::ADD;
        appendBoxEdges(crop_box,
                       preprocessor_config_.crop_box_min_m.cast<double>(),
                       preprocessor_config_.crop_box_max_m.cast<double>());
    } else {
        crop_box.action = visualization_msgs::msg::Marker::DELETE;
    }
    markers.markers.push_back(crop_box);

    const auto append_initial_origin = [&markers,
                                        &lidar_header](const Eigen::Vector3d& position,
                                                       const std::string& label,
                                                       const std::int32_t sphere_id,
                                                       const std::int32_t text_id,
                                                       const std::array<float, 3>& color) {
        visualization_msgs::msg::Marker sphere;
        sphere.header = lidar_header;
        sphere.ns = "mid70_initial_origins";
        sphere.id = sphere_id;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position = pointMessage(position);
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = 0.10;
        sphere.scale.y = 0.10;
        sphere.scale.z = 0.10;
        sphere.color.r = color[0];
        sphere.color.g = color[1];
        sphere.color.b = color[2];
        sphere.color.a = 1.0F;
        markers.markers.push_back(sphere);

        visualization_msgs::msg::Marker text;
        text.header = lidar_header;
        text.ns = "mid70_initial_origins";
        text.id = text_id;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position = pointMessage(position + Eigen::Vector3d(0.0, 0.0, 0.16));
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.10;
        text.color.r = color[0];
        text.color.g = color[1];
        text.color.b = color[2];
        text.color.a = 1.0F;
        text.text = label;
        markers.markers.push_back(text);
    };

    if (isFiniteTransform(t_lidar_base_initial_)) {
        append_initial_origin(
            t_lidar_base_initial_.translation(), "BASE INITIAL ORIGIN", 0, 1, {0.2F, 0.55F, 1.0F});
        const Eigen::Isometry3d t_lidar_armor_initial = t_lidar_base_initial_ * t_base_armor_zero_;
        if (isFiniteTransform(t_lidar_armor_initial)) {
            append_initial_origin(t_lidar_armor_initial.translation(),
                                  "ARMOR INITIAL ORIGIN",
                                  2,
                                  3,
                                  {1.0F, 0.45F, 0.05F});
        }
    }

    markers_publisher_->publish(markers);
}

} // namespace dart_vision::lidar
