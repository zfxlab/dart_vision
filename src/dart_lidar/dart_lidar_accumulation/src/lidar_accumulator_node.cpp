#include "dart_lidar_accumulation/lidar_accumulator_node.hpp"

#include <cmath>
#include <functional>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <utility>

namespace dart_vision::lidar {
namespace {

void finalizeCloud(LidarAccumulatorNode::PointCloud& cloud) {
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1U;
    cloud.is_dense = true;
}

std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
}

rcl_interfaces::msg::SetParametersResult parameterFailure(const std::string& reason) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
}

} // namespace

LidarAccumulatorNode::LidarAccumulatorNode(const rclcpp::NodeOptions& options)
    : Node("lidar_accumulator_node", options) {
    declareParameters();
    config_ = readConfig();
    if (const std::string error = config_.validationError(); !error.empty()) {
        throw std::invalid_argument("Invalid LiDAR accumulator parameter: " + error);
    }

    accumulated_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.input_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&LidarAccumulatorNode::cloudCallback, this, std::placeholders::_1));
    parameter_callback_ = add_on_set_parameters_callback(
        std::bind(&LidarAccumulatorNode::onParametersChanged, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Sliding point-cloud accumulation %s -> %s in %s: %.0fms, %zu..%zu frames, "
                "publish %.2fHz after %zu new frames",
                config_.input_topic.c_str(),
                config_.output_topic.c_str(),
                config_.accumulation_frame.c_str(),
                config_.accumulation.window_duration_s * 1000.0,
                config_.accumulation.min_frames,
                config_.accumulation.max_frames,
                config_.publish.rate_hz,
                config_.publish.min_new_frames);
}

void LidarAccumulatorNode::declareParameters() {
    const LidarAccumulatorConfig defaults;
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "Loaded at startup; restart the node to change this parameter";
    declare_parameter<std::string>("input_topic", defaults.input_topic, read_only);
    declare_parameter<std::string>("output_topic", defaults.output_topic, read_only);
    declare_parameter<std::string>("accumulation_frame", defaults.accumulation_frame, read_only);
    declare_parameter<bool>("transform.crop_box_enabled", defaults.transform.crop_box_enabled);
    declare_parameter<std::vector<double>>("transform.crop_box_min_m",
                                           {defaults.transform.crop_box_min_m.x(),
                                            defaults.transform.crop_box_min_m.y(),
                                            defaults.transform.crop_box_min_m.z()});
    declare_parameter<std::vector<double>>("transform.crop_box_max_m",
                                           {defaults.transform.crop_box_max_m.x(),
                                            defaults.transform.crop_box_max_m.y(),
                                            defaults.transform.crop_box_max_m.z()});
    declare_parameter<double>("accumulation.window_duration_s",
                              defaults.accumulation.window_duration_s);
    declare_parameter<std::int64_t>("accumulation.min_frames",
                                    static_cast<std::int64_t>(defaults.accumulation.min_frames));
    declare_parameter<std::int64_t>("accumulation.max_frames",
                                    static_cast<std::int64_t>(defaults.accumulation.max_frames));
    declare_parameter<std::int64_t>("accumulation.max_points",
                                    static_cast<std::int64_t>(defaults.accumulation.max_points));
    declare_parameter<bool>("accumulation.output_voxel_grid_enabled",
                            defaults.accumulation.output_voxel_grid_enabled);
    declare_parameter<double>("accumulation.output_voxel_leaf_size_m",
                              defaults.accumulation.output_voxel_leaf_size_m);
    declare_parameter<double>("publish.rate_hz", defaults.publish.rate_hz);
    declare_parameter<std::int64_t>("publish.min_new_frames",
                                    static_cast<std::int64_t>(defaults.publish.min_new_frames));
    declare_parameter<bool>("publish.immediately_when_ready",
                            defaults.publish.immediately_when_ready);
}

LidarAccumulatorConfig LidarAccumulatorNode::readConfig() const {
    LidarAccumulatorConfig config;
    config.input_topic = get_parameter("input_topic").as_string();
    config.output_topic = get_parameter("output_topic").as_string();
    config.accumulation_frame = get_parameter("accumulation_frame").as_string();
    config.transform.crop_box_enabled = get_parameter("transform.crop_box_enabled").as_bool();
    config.transform.crop_box_min_m = readVector3("transform.crop_box_min_m");
    config.transform.crop_box_max_m = readVector3("transform.crop_box_max_m");
    config.accumulation.window_duration_s =
        get_parameter("accumulation.window_duration_s").as_double();
    const auto min_frames = get_parameter("accumulation.min_frames").as_int();
    const auto max_frames = get_parameter("accumulation.max_frames").as_int();
    const auto max_points = get_parameter("accumulation.max_points").as_int();
    config.accumulation.min_frames = min_frames > 0 ? static_cast<std::size_t>(min_frames) : 0U;
    config.accumulation.max_frames = max_frames > 0 ? static_cast<std::size_t>(max_frames) : 0U;
    config.accumulation.max_points = max_points > 0 ? static_cast<std::size_t>(max_points) : 0U;
    config.accumulation.output_voxel_grid_enabled =
        get_parameter("accumulation.output_voxel_grid_enabled").as_bool();
    config.accumulation.output_voxel_leaf_size_m =
        get_parameter("accumulation.output_voxel_leaf_size_m").as_double();
    config.publish.rate_hz = get_parameter("publish.rate_hz").as_double();
    const auto min_new_frames = get_parameter("publish.min_new_frames").as_int();
    config.publish.min_new_frames =
        min_new_frames > 0 ? static_cast<std::size_t>(min_new_frames) : 0U;
    config.publish.immediately_when_ready =
        get_parameter("publish.immediately_when_ready").as_bool();
    return config;
}

Eigen::Vector3f LidarAccumulatorNode::readVector3(const std::string& name) const {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3U) {
        throw std::invalid_argument(name + " must contain exactly three values");
    }
    return Eigen::Vector3f(static_cast<float>(values[0]),
                           static_cast<float>(values[1]),
                           static_cast<float>(values[2]));
}

rcl_interfaces::msg::SetParametersResult
LidarAccumulatorNode::onParametersChanged(const std::vector<rclcpp::Parameter>& parameters) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    LidarAccumulatorConfig updated = config_;
    bool changed = false;
    try {
        for (const auto& parameter : parameters) {
            const std::string& name = parameter.get_name();
            if (name == "input_topic" || name == "output_topic" || name == "accumulation_frame") {
                return parameterFailure(name + " cannot be changed while the node is running");
            }
            if (name == "transform.crop_box_enabled") {
                updated.transform.crop_box_enabled = parameter.as_bool();
                changed = true;
            } else if (name == "transform.crop_box_min_m" || name == "transform.crop_box_max_m") {
                const auto values = parameter.as_double_array();
                if (values.size() != 3U) {
                    return parameterFailure(name + " must contain exactly three values");
                }
                const Eigen::Vector3f value(static_cast<float>(values[0]),
                                            static_cast<float>(values[1]),
                                            static_cast<float>(values[2]));
                if (name == "transform.crop_box_min_m") {
                    updated.transform.crop_box_min_m = value;
                } else {
                    updated.transform.crop_box_max_m = value;
                }
                changed = true;
            } else if (name == "accumulation.window_duration_s") {
                updated.accumulation.window_duration_s = parameter.as_double();
                changed = true;
            } else if (name == "accumulation.min_frames" || name == "accumulation.max_frames" ||
                       name == "accumulation.max_points") {
                const auto value = parameter.as_int();
                if (value <= 0) {
                    return parameterFailure(name + " must be positive");
                }
                if (name == "accumulation.min_frames") {
                    updated.accumulation.min_frames = static_cast<std::size_t>(value);
                } else if (name == "accumulation.max_frames") {
                    updated.accumulation.max_frames = static_cast<std::size_t>(value);
                } else {
                    updated.accumulation.max_points = static_cast<std::size_t>(value);
                }
                changed = true;
            } else if (name == "accumulation.output_voxel_grid_enabled") {
                updated.accumulation.output_voxel_grid_enabled = parameter.as_bool();
                changed = true;
            } else if (name == "accumulation.output_voxel_leaf_size_m") {
                updated.accumulation.output_voxel_leaf_size_m = parameter.as_double();
                changed = true;
            } else if (name == "publish.rate_hz") {
                updated.publish.rate_hz = parameter.as_double();
                changed = true;
            } else if (name == "publish.min_new_frames") {
                const auto value = parameter.as_int();
                if (value <= 0) {
                    return parameterFailure(name + " must be positive");
                }
                updated.publish.min_new_frames = static_cast<std::size_t>(value);
                changed = true;
            } else if (name == "publish.immediately_when_ready") {
                updated.publish.immediately_when_ready = parameter.as_bool();
                changed = true;
            }
        }
    } catch (const rclcpp::ParameterTypeException& error) {
        return parameterFailure(error.what());
    }
    if (!changed) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }
    if (const std::string error = updated.validationError(); !error.empty()) {
        return parameterFailure(error);
    }
    config_ = std::move(updated);
    if (!frames_.empty()) {
        pruneWindow(last_stamp_ns_);
    }
    RCLCPP_INFO(get_logger(), "Sliding accumulator parameters applied");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void LidarAccumulatorNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    ++received_clouds_;
    const std::int64_t stamp_ns = stampNanoseconds(message->header.stamp);
    if (stamp_ns <= 0 || message->header.frame_id != config_.accumulation_frame) {
        ++dropped_clouds_;
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             1000,
                             "Dropping cloud: expected positive timestamp in frame %s, got %s",
                             config_.accumulation_frame.c_str(),
                             message->header.frame_id.c_str());
        return;
    }
    if (last_stamp_ns_ > 0 && stamp_ns < last_stamp_ns_) {
        RCLCPP_WARN(get_logger(),
                    "Point-cloud time moved backwards; resetting accumulation window");
        resetWindow();
    }
    last_stamp_ns_ = stamp_ns;

    PointCloud::Ptr cloud = cropCloud(*message);
    if (cloud->empty()) {
        ++dropped_clouds_;
        return;
    }
    addFrame(std::move(cloud), stamp_ns, message->header.stamp);
    pruneWindow(stamp_ns);
    ++accepted_clouds_;
    ++new_frames_since_publish_;

    if (publicationDue(stamp_ns)) {
        publishWindow();
    }
}

LidarAccumulatorNode::PointCloud::Ptr
LidarAccumulatorNode::cropCloud(const sensor_msgs::msg::PointCloud2& message) const {
    PointCloud::Ptr source(new PointCloud);
    pcl::fromROSMsg(message, *source);
    PointCloud::Ptr filtered(new PointCloud);
    filtered->reserve(source->size());
    for (const auto& point : *source) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        const Eigen::Vector3f value(point.x, point.y, point.z);
        if (config_.transform.crop_box_enabled &&
            ((value.array() < config_.transform.crop_box_min_m.array()).any() ||
             (value.array() > config_.transform.crop_box_max_m.array()).any())) {
            continue;
        }
        filtered->push_back(point);
    }
    finalizeCloud(*filtered);
    return filtered;
}

void LidarAccumulatorNode::addFrame(PointCloud::Ptr cloud,
                                    const std::int64_t stamp_ns,
                                    const builtin_interfaces::msg::Time& stamp) {
    if (cloud->size() > config_.accumulation.max_points) {
        cloud->resize(config_.accumulation.max_points);
        finalizeCloud(*cloud);
    }
    while (!frames_.empty() &&
           accumulated_points_ > config_.accumulation.max_points - cloud->size()) {
        accumulated_points_ -= frames_.front().cloud->size();
        frames_.pop_front();
    }
    accumulated_points_ += cloud->size();
    frames_.push_back(StampedCloud{std::move(cloud), stamp_ns, stamp});
}

void LidarAccumulatorNode::pruneWindow(const std::int64_t newest_stamp_ns) {
    const std::int64_t maximum_age_ns =
        static_cast<std::int64_t>(std::llround(config_.accumulation.window_duration_s * 1.0e9));
    while (!frames_.empty() && (newest_stamp_ns - frames_.front().stamp_ns > maximum_age_ns ||
                                frames_.size() > config_.accumulation.max_frames)) {
        accumulated_points_ -= frames_.front().cloud->size();
        frames_.pop_front();
    }
}

bool LidarAccumulatorNode::publicationDue(const std::int64_t newest_stamp_ns) const {
    if (frames_.size() < config_.accumulation.min_frames) {
        return false;
    }
    if (!has_published_) {
        if (config_.publish.immediately_when_ready) {
            return true;
        }
        return newest_stamp_ns - frames_.front().stamp_ns >=
               static_cast<std::int64_t>(std::llround(1.0e9 / config_.publish.rate_hz));
    }
    const std::int64_t publish_period_ns =
        static_cast<std::int64_t>(std::llround(1.0e9 / config_.publish.rate_hz));
    return newest_stamp_ns - last_publish_stamp_ns_ >= publish_period_ns &&
           new_frames_since_publish_ >= config_.publish.min_new_frames;
}

void LidarAccumulatorNode::publishWindow() {
    PointCloud::Ptr accumulated(new PointCloud);
    accumulated->reserve(accumulated_points_);
    for (const auto& frame : frames_) {
        accumulated->insert(accumulated->end(), frame.cloud->begin(), frame.cloud->end());
    }
    finalizeCloud(*accumulated);
    if (config_.accumulation.output_voxel_grid_enabled && !accumulated->empty()) {
        PointCloud::Ptr downsampled(new PointCloud);
        pcl::VoxelGrid<PointT> voxel;
        const float leaf = static_cast<float>(config_.accumulation.output_voxel_leaf_size_m);
        voxel.setInputCloud(accumulated);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*downsampled);
        finalizeCloud(*downsampled);
        accumulated = std::move(downsampled);
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*accumulated, message);
    message.header.frame_id = config_.accumulation_frame;
    message.header.stamp = frames_.back().stamp;
    accumulated_publisher_->publish(message);
    last_publish_stamp_ns_ = frames_.back().stamp_ns;
    new_frames_since_publish_ = 0U;
    has_published_ = true;
    ++published_clouds_;
    RCLCPP_INFO(get_logger(),
                "LiDAR sliding window published: %zu frames, %zu input points, %zu output points; "
                "received=%llu accepted=%llu dropped=%llu published=%llu",
                frames_.size(),
                accumulated_points_,
                accumulated->size(),
                static_cast<unsigned long long>(received_clouds_),
                static_cast<unsigned long long>(accepted_clouds_),
                static_cast<unsigned long long>(dropped_clouds_),
                static_cast<unsigned long long>(published_clouds_));
}

void LidarAccumulatorNode::resetWindow() {
    frames_.clear();
    accumulated_points_ = 0U;
    last_stamp_ns_ = 0;
    last_publish_stamp_ns_ = 0;
    new_frames_since_publish_ = 0U;
    has_published_ = false;
}

} // namespace dart_vision::lidar
