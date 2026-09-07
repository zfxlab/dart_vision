#include "dart_lidar_accumulation/lidar_accumulator_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utility>
#include <vector>

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
    : Node("lidar_accumulator_node", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    declareParameters();
    config_ = readConfig();
    if (const std::string error = config_.validationError(); !error.empty()) {
        throw std::invalid_argument("Invalid LiDAR accumulator parameter: " + error);
    }

    accumulated_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.input_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&LidarAccumulatorNode::cloudCallback, this, std::placeholders::_1));

    if (config_.mode == RuntimeMode::kTriggeredOnline) {
        controller_state_subscription_ = create_subscription<dart_interfaces::msg::ControllerState>(
            config_.controller_state_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&LidarAccumulatorNode::controllerStateCallback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(),
                    "Online gated accumulation: yaw_hold=%.0fms post_delay=%.0fms "
                    "accumulate=%.0fms frames=%zu..%zu deadline=%.0fms",
                    config_.stability.hold_s * 1000.0,
                    config_.stability.post_stable_delay_s * 1000.0,
                    config_.accumulation.duration_s * 1000.0,
                    config_.accumulation.min_frames,
                    config_.accumulation.max_frames,
                    config_.stability.measurement_deadline_s * 1000.0);
    } else {
        RCLCPP_INFO(get_logger(),
                    "Offline bag accumulation: source-time windows=%.2fs frames=%zu..%zu",
                    config_.accumulation.duration_s,
                    config_.accumulation.min_frames,
                    config_.accumulation.max_frames);
    }
    RCLCPP_INFO(get_logger(),
                "PointCloud2 accumulation %s -> %s in %s",
                config_.input_topic.c_str(),
                config_.output_topic.c_str(),
                config_.accumulation_frame.c_str());

    parameter_callback_ = add_on_set_parameters_callback(
        std::bind(&LidarAccumulatorNode::onParametersChanged, this, std::placeholders::_1));
}

void LidarAccumulatorNode::declareParameters() {
    const LidarAccumulatorConfig defaults;
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "Loaded at startup; restart the node to change this parameter";

    declare_parameter<std::string>("mode", defaults.mode_name, read_only);
    declare_parameter<std::string>("input_topic", defaults.input_topic, read_only);
    declare_parameter<std::string>("output_topic", defaults.output_topic, read_only);
    declare_parameter<std::string>(
        "controller_state_topic", defaults.controller_state_topic, read_only);
    declare_parameter<std::string>("accumulation_frame", defaults.accumulation_frame, read_only);

    declare_parameter<double>("stability.hold_s", defaults.stability.hold_s);
    declare_parameter<double>("stability.yaw_span_rad", defaults.stability.yaw_span_rad);
    declare_parameter<double>("stability.abort_yaw_deviation_rad",
                              defaults.stability.abort_yaw_deviation_rad);
    declare_parameter<double>("stability.controller_timeout_s",
                              defaults.stability.controller_timeout_s);
    declare_parameter<double>("stability.post_stable_delay_s",
                              defaults.stability.post_stable_delay_s);
    declare_parameter<double>("stability.measurement_deadline_s",
                              defaults.stability.measurement_deadline_s);
    declare_parameter<double>("stability.localization_reserve_s",
                              defaults.stability.localization_reserve_s);

    declare_parameter<bool>("transform.crop_box_enabled", defaults.transform.crop_box_enabled);
    declare_parameter<std::vector<double>>("transform.crop_box_min_m",
                                           {defaults.transform.crop_box_min_m.x(),
                                            defaults.transform.crop_box_min_m.y(),
                                            defaults.transform.crop_box_min_m.z()});
    declare_parameter<std::vector<double>>("transform.crop_box_max_m",
                                           {defaults.transform.crop_box_max_m.x(),
                                            defaults.transform.crop_box_max_m.y(),
                                            defaults.transform.crop_box_max_m.z()});

    declare_parameter<double>("accumulation.duration_s", defaults.accumulation.duration_s);
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
}

LidarAccumulatorConfig LidarAccumulatorNode::readConfig() const {
    LidarAccumulatorConfig config;
    config.mode_name = get_parameter("mode").as_string();
    if (config.mode_name == "triggered_online") {
        config.mode = RuntimeMode::kTriggeredOnline;
    } else if (config.mode_name == "bag_offline") {
        config.mode = RuntimeMode::kBagOffline;
    }
    config.input_topic = get_parameter("input_topic").as_string();
    config.output_topic = get_parameter("output_topic").as_string();
    config.controller_state_topic = get_parameter("controller_state_topic").as_string();
    config.accumulation_frame = get_parameter("accumulation_frame").as_string();

    config.stability.hold_s = get_parameter("stability.hold_s").as_double();
    config.stability.yaw_span_rad = get_parameter("stability.yaw_span_rad").as_double();
    config.stability.abort_yaw_deviation_rad =
        get_parameter("stability.abort_yaw_deviation_rad").as_double();
    config.stability.controller_timeout_s =
        get_parameter("stability.controller_timeout_s").as_double();
    config.stability.post_stable_delay_s =
        get_parameter("stability.post_stable_delay_s").as_double();
    config.stability.measurement_deadline_s =
        get_parameter("stability.measurement_deadline_s").as_double();
    config.stability.localization_reserve_s =
        get_parameter("stability.localization_reserve_s").as_double();

    config.transform.crop_box_enabled = get_parameter("transform.crop_box_enabled").as_bool();
    config.transform.crop_box_min_m = readVector3("transform.crop_box_min_m");
    config.transform.crop_box_max_m = readVector3("transform.crop_box_max_m");

    config.accumulation.duration_s = get_parameter("accumulation.duration_s").as_double();
    const auto min_frames = get_parameter("accumulation.min_frames").as_int();
    const auto max_frames = get_parameter("accumulation.max_frames").as_int();
    const auto max_points = get_parameter("accumulation.max_points").as_int();
    if (min_frames > 0) {
        config.accumulation.min_frames = static_cast<std::size_t>(min_frames);
    }
    if (max_frames >= 0) {
        config.accumulation.max_frames = static_cast<std::size_t>(max_frames);
    }
    if (max_points > 0) {
        config.accumulation.max_points = static_cast<std::size_t>(max_points);
    }
    config.accumulation.output_voxel_grid_enabled =
        get_parameter("accumulation.output_voxel_grid_enabled").as_bool();
    config.accumulation.output_voxel_leaf_size_m =
        get_parameter("accumulation.output_voxel_leaf_size_m").as_double();
    if (min_frames <= 0) {
        config.accumulation.min_frames = 0U;
    }
    if (max_frames < 0) {
        config.accumulation.max_frames = 0U;
    }
    if (max_points <= 0) {
        config.accumulation.max_points = 0U;
    }
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
    LidarAccumulatorConfig updated = pending_config_.value_or(config_);
    bool config_changed = false;

    try {
        for (const auto& parameter : parameters) {
            const std::string& name = parameter.get_name();
            if (name == "mode" || name == "input_topic" || name == "output_topic" ||
                name == "controller_state_topic" || name == "accumulation_frame") {
                return parameterFailure(name + " cannot be changed while the node is running");
            }

            if (name == "stability.hold_s") {
                updated.stability.hold_s = parameter.as_double();
                config_changed = true;
            } else if (name == "stability.yaw_span_rad") {
                updated.stability.yaw_span_rad = parameter.as_double();
                config_changed = true;
            } else if (name == "stability.abort_yaw_deviation_rad") {
                updated.stability.abort_yaw_deviation_rad = parameter.as_double();
                config_changed = true;
            } else if (name == "stability.controller_timeout_s") {
                updated.stability.controller_timeout_s = parameter.as_double();
                config_changed = true;
            } else if (name == "stability.post_stable_delay_s") {
                updated.stability.post_stable_delay_s = parameter.as_double();
                config_changed = true;
            } else if (name == "stability.measurement_deadline_s") {
                updated.stability.measurement_deadline_s = parameter.as_double();
                config_changed = true;
            } else if (name == "stability.localization_reserve_s") {
                updated.stability.localization_reserve_s = parameter.as_double();
                config_changed = true;
            } else if (name == "transform.crop_box_enabled") {
                updated.transform.crop_box_enabled = parameter.as_bool();
                config_changed = true;
            } else if (name == "transform.crop_box_min_m" || name == "transform.crop_box_max_m") {
                const auto values = parameter.as_double_array();
                if (values.size() != 3U) {
                    return parameterFailure(name + " must contain exactly three values");
                }
                Eigen::Vector3f value(static_cast<float>(values[0]),
                                      static_cast<float>(values[1]),
                                      static_cast<float>(values[2]));
                if (name == "transform.crop_box_min_m") {
                    updated.transform.crop_box_min_m = value;
                } else {
                    updated.transform.crop_box_max_m = value;
                }
                config_changed = true;
            } else if (name == "accumulation.duration_s") {
                updated.accumulation.duration_s = parameter.as_double();
                config_changed = true;
            } else if (name == "accumulation.min_frames" || name == "accumulation.max_frames" ||
                       name == "accumulation.max_points") {
                const std::int64_t value = parameter.as_int();
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
                config_changed = true;
            } else if (name == "accumulation.output_voxel_grid_enabled") {
                updated.accumulation.output_voxel_grid_enabled = parameter.as_bool();
                config_changed = true;
            } else if (name == "accumulation.output_voxel_leaf_size_m") {
                updated.accumulation.output_voxel_leaf_size_m = parameter.as_double();
                config_changed = true;
            }
        }
    } catch (const rclcpp::ParameterTypeException& error) {
        return parameterFailure(error.what());
    }

    if (!config_changed) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    if (const std::string error = updated.validationError(); !error.empty()) {
        return parameterFailure(error);
    }

    const bool measurement_active =
        state_ == MeasurementState::kSettling || state_ == MeasurementState::kAccumulating;
    if (measurement_active) {
        pending_config_ = std::move(updated);
        RCLCPP_INFO(get_logger(), "LiDAR parameter update staged for the next measurement window");
    } else {
        config_ = std::move(updated);
        pending_config_.reset();
        RCLCPP_INFO(get_logger(), "LiDAR parameter update applied");
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void LidarAccumulatorNode::applyPendingConfigIfIdle() {
    const bool measurement_active =
        state_ == MeasurementState::kSettling || state_ == MeasurementState::kAccumulating;
    if (!measurement_active && pending_config_) {
        config_ = std::move(*pending_config_);
        pending_config_.reset();
        RCLCPP_INFO(get_logger(), "Staged LiDAR parameters applied at a measurement boundary");
    }
}

void LidarAccumulatorNode::controllerStateCallback(
    const dart_interfaces::msg::ControllerState::ConstSharedPtr& message) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    applyPendingConfigIfIdle();
    const double yaw_rad = static_cast<double>(message->yaw_rad);
    if (!std::isfinite(yaw_rad)) {
        return;
    }

    const SteadyTime now = std::chrono::steady_clock::now();
    have_controller_update_ = true;
    last_controller_update_ = now;
    yaw_samples_.push_back(YawSample{now, yaw_rad});

    while (yaw_samples_.size() > 2U &&
           std::chrono::duration<double>(now - yaw_samples_[1].received_at).count() >=
               config_.stability.hold_s) {
        yaw_samples_.pop_front();
    }

    const auto [minimum, maximum] = std::minmax_element(
        yaw_samples_.begin(), yaw_samples_.end(), [](const YawSample& lhs, const YawSample& rhs) {
            return lhs.yaw_rad < rhs.yaw_rad;
        });
    const double window_duration_s =
        std::chrono::duration<double>(now - yaw_samples_.front().received_at).count();
    const bool yaw_stable = window_duration_s >= config_.stability.hold_s &&
                            maximum->yaw_rad - minimum->yaw_rad <= config_.stability.yaw_span_rad;

    if ((state_ == MeasurementState::kSettling || state_ == MeasurementState::kAccumulating) &&
        std::abs(yaw_rad - frozen_yaw_rad_) > config_.stability.abort_yaw_deviation_rad) {
        abortMeasurement("yaw moved away from the frozen position", true);
        return;
    }

    if (!yaw_stable) {
        if (state_ == MeasurementState::kSettling || state_ == MeasurementState::kAccumulating) {
            abortMeasurement("yaw stability was lost", true);
        } else if (state_ == MeasurementState::kComplete) {
            state_ = MeasurementState::kWaitingForYaw;
            yaw_samples_.clear();
        }
        return;
    }

    if (state_ == MeasurementState::kWaitingForYaw) {
        startMeasurement(yaw_samples_.front().received_at, now, yaw_rad);
    }
}

void LidarAccumulatorNode::startMeasurement(const SteadyTime stable_window_start,
                                            const SteadyTime stable_declared_at,
                                            const double yaw_rad) {
    clearAccumulation();
    frozen_transform_ready_ = false;
    frozen_input_frame_.clear();
    frozen_yaw_rad_ = yaw_rad;
    measurement_origin_ = stable_window_start;
    accumulation_not_before_ =
        stable_declared_at +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(config_.stability.post_stable_delay_s));
    measurement_deadline_ =
        stable_window_start +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(config_.stability.measurement_deadline_s));
    state_ = MeasurementState::kSettling;
    RCLCPP_INFO(get_logger(),
                "Yaw stable at %.6f rad; accepting clouds after %.0fms",
                frozen_yaw_rad_,
                config_.stability.post_stable_delay_s * 1000.0);
}

void LidarAccumulatorNode::abortMeasurement(const char* reason, const bool clear_yaw_history) {
    if (state_ == MeasurementState::kSettling || state_ == MeasurementState::kAccumulating) {
        ++aborted_measurements_;
        RCLCPP_WARN(get_logger(), "Aborting LiDAR measurement: %s", reason);
    }
    clearAccumulation();
    frozen_transform_ready_ = false;
    frozen_input_frame_.clear();
    offline_window_start_ns_ = 0;
    offline_last_stamp_ns_ = 0;
    state_ = MeasurementState::kWaitingForYaw;
    if (clear_yaw_history) {
        yaw_samples_.clear();
    }
}

bool LidarAccumulatorNode::freezeTransform(const std::string& input_frame) {
    if (input_frame == config_.accumulation_frame) {
        frozen_transform_ = Eigen::Isometry3d::Identity();
    } else {
        try {
            const auto transform =
                tf_buffer_.lookupTransform(config_.accumulation_frame,
                                           input_frame,
                                           rclcpp::Time(0, 0, get_clock()->get_clock_type()),
                                           rclcpp::Duration::from_seconds(0.0));
            frozen_transform_ = tf2::transformToEigen(transform.transform);
        } catch (const tf2::TransformException& error) {
            ++dropped_tf_clouds_;
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 1000,
                                 "Unable to freeze latest TF %s <- %s: %s",
                                 config_.accumulation_frame.c_str(),
                                 input_frame.c_str(),
                                 error.what());
            return false;
        }
    }
    frozen_input_frame_ = input_frame;
    frozen_transform_ready_ = true;
    RCLCPP_INFO(get_logger(),
                "Frozen TF %s <- %s for a new accumulation window",
                config_.accumulation_frame.c_str(),
                input_frame.c_str());
    return true;
}

void LidarAccumulatorNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    applyPendingConfigIfIdle();
    ++received_clouds_;
    const SteadyTime received_at = std::chrono::steady_clock::now();
    if (config_.mode == RuntimeMode::kTriggeredOnline) {
        handleOnlineCloud(*message, received_at);
    } else {
        handleOfflineCloud(*message, received_at);
    }
}

void LidarAccumulatorNode::handleOnlineCloud(const sensor_msgs::msg::PointCloud2& message,
                                             const SteadyTime received_at) {
    if (state_ == MeasurementState::kWaitingForYaw || state_ == MeasurementState::kComplete) {
        ++ignored_clouds_;
        return;
    }
    if (!have_controller_update_ ||
        std::chrono::duration<double>(received_at - last_controller_update_).count() >
            config_.stability.controller_timeout_s) {
        abortMeasurement("controller yaw data timed out", true);
        return;
    }
    if (received_at >= measurement_deadline_) {
        abortMeasurement("one-second measurement deadline expired", false);
        return;
    }
    if (state_ == MeasurementState::kSettling && received_at < accumulation_not_before_) {
        ++ignored_clouds_;
        return;
    }
    if (message.header.frame_id.empty()) {
        ++dropped_tf_clouds_;
        return;
    }

    const auto reserve_duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(config_.stability.localization_reserve_s));
    if (state_ == MeasurementState::kSettling) {
        if (received_at >= measurement_deadline_ - reserve_duration) {
            abortMeasurement("not enough time remains for localization", false);
            return;
        }
        if (!freezeTransform(message.header.frame_id)) {
            return;
        }
        accumulation_started_at_ = received_at;
        state_ = MeasurementState::kAccumulating;
    }

    if (message.header.frame_id != frozen_input_frame_) {
        abortMeasurement("point-cloud frame_id changed during accumulation", true);
        return;
    }

    PointCloud::Ptr cloud = transformAndCrop(message);
    if (cloud->empty()) {
        ++dropped_empty_clouds_;
        return;
    }
    last_cloud_stamp_ = message.header.stamp;
    addFrame(std::move(cloud));
    ++accepted_clouds_;

    const double accumulated_duration_s =
        std::chrono::duration<double>(received_at - accumulation_started_at_).count();
    const bool normal_completion = frames_.size() >= config_.accumulation.min_frames &&
                                   (frames_.size() >= config_.accumulation.max_frames ||
                                    accumulated_duration_s >= config_.accumulation.duration_s);
    const bool localization_deadline_reached =
        received_at >= measurement_deadline_ - reserve_duration;

    if (normal_completion) {
        finalizeAndPublish(received_at);
    } else if (localization_deadline_reached) {
        if (frames_.size() >= config_.accumulation.min_frames) {
            finalizeAndPublish(received_at);
        } else {
            abortMeasurement("insufficient LiDAR frames before localization reserve", false);
        }
    }
}

void LidarAccumulatorNode::handleOfflineCloud(const sensor_msgs::msg::PointCloud2& message,
                                              const SteadyTime received_at) {
    if (message.header.frame_id.empty()) {
        ++dropped_tf_clouds_;
        return;
    }
    const std::int64_t source_stamp_ns = stampNanoseconds(message.header.stamp);
    if (source_stamp_ns <= 0) {
        ++dropped_empty_clouds_;
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             1000,
                             "bag_offline requires a positive monotonic source timestamp");
        return;
    }

    if (!frozen_transform_ready_) {
        if (!freezeTransform(message.header.frame_id)) {
            return;
        }
        clearAccumulation();
        offline_window_start_ns_ = source_stamp_ns;
        state_ = MeasurementState::kAccumulating;
    } else if (message.header.frame_id != frozen_input_frame_) {
        abortMeasurement("point-cloud frame_id changed during offline window", false);
        return;
    }

    if (offline_last_stamp_ns_ > 0 && source_stamp_ns < offline_last_stamp_ns_) {
        abortMeasurement("source time moved backwards during offline window", false);
        return;
    }
    offline_last_stamp_ns_ = source_stamp_ns;

    PointCloud::Ptr cloud = transformAndCrop(message);
    if (cloud->empty()) {
        ++dropped_empty_clouds_;
        return;
    }
    last_cloud_stamp_ = message.header.stamp;
    addFrame(std::move(cloud));
    ++accepted_clouds_;

    const double source_duration_s =
        static_cast<double>(source_stamp_ns - offline_window_start_ns_) * 1.0e-9;
    if (frames_.size() >= config_.accumulation.min_frames &&
        (frames_.size() >= config_.accumulation.max_frames ||
         source_duration_s >= config_.accumulation.duration_s)) {
        finalizeAndPublish(received_at);
    }
}

LidarAccumulatorNode::PointCloud::Ptr
LidarAccumulatorNode::transformAndCrop(const sensor_msgs::msg::PointCloud2& message) const {
    PointCloud::Ptr source(new PointCloud);
    pcl::fromROSMsg(message, *source);
    PointCloud::Ptr filtered(new PointCloud);
    filtered->reserve(source->size());

    for (const auto& point : *source) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        const Eigen::Vector3d transformed =
            frozen_transform_ * Eigen::Vector3d(point.x, point.y, point.z);
        if (!transformed.allFinite()) {
            continue;
        }
        const Eigen::Vector3f transformed_f = transformed.cast<float>();
        if (config_.transform.crop_box_enabled &&
            ((transformed_f.array() < config_.transform.crop_box_min_m.array()).any() ||
             (transformed_f.array() > config_.transform.crop_box_max_m.array()).any())) {
            continue;
        }

        PointT output = point;
        output.x = transformed_f.x();
        output.y = transformed_f.y();
        output.z = transformed_f.z();
        filtered->push_back(output);
    }
    finalizeCloud(*filtered);
    return filtered;
}

void LidarAccumulatorNode::addFrame(PointCloud::Ptr cloud) {
    if (cloud->size() > config_.accumulation.max_points) {
        cloud->resize(config_.accumulation.max_points);
        finalizeCloud(*cloud);
    }
    while (!frames_.empty() &&
           accumulated_points_ > config_.accumulation.max_points - cloud->size()) {
        accumulated_points_ -= frames_.front()->size();
        frames_.pop_front();
    }
    accumulated_points_ += cloud->size();
    frames_.push_back(std::move(cloud));
}

void LidarAccumulatorNode::finalizeAndPublish(const SteadyTime completed_at) {
    PointCloud::Ptr accumulated(new PointCloud);
    accumulated->reserve(accumulated_points_);
    for (const auto& frame : frames_) {
        accumulated->insert(accumulated->end(), frame->begin(), frame->end());
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
    if (config_.mode == RuntimeMode::kTriggeredOnline) {
        message.header.stamp = now();
    } else {
        message.header.stamp = last_cloud_stamp_;
    }
    accumulated_publisher_->publish(message);

    ++completed_measurements_;
    const double latency_or_window_ms =
        config_.mode == RuntimeMode::kTriggeredOnline
            ? std::chrono::duration<double, std::milli>(completed_at - measurement_origin_).count()
            : static_cast<double>(offline_last_stamp_ns_ - offline_window_start_ns_) * 1.0e-6;
    RCLCPP_INFO(get_logger(),
                "LiDAR window complete in %.1fms: %zu frames, %zu input points, %zu output "
                "points; received=%llu ignored=%llu accepted=%llu aborted=%llu completed=%llu",
                latency_or_window_ms,
                frames_.size(),
                accumulated_points_,
                accumulated->size(),
                static_cast<unsigned long long>(received_clouds_),
                static_cast<unsigned long long>(ignored_clouds_),
                static_cast<unsigned long long>(accepted_clouds_),
                static_cast<unsigned long long>(aborted_measurements_),
                static_cast<unsigned long long>(completed_measurements_));

    clearAccumulation();
    if (config_.mode == RuntimeMode::kTriggeredOnline) {
        state_ = MeasurementState::kComplete;
    } else {
        frozen_transform_ready_ = false;
        frozen_input_frame_.clear();
        offline_window_start_ns_ = 0;
        offline_last_stamp_ns_ = 0;
        state_ = MeasurementState::kWaitingForYaw;
    }
}

void LidarAccumulatorNode::clearAccumulation() {
    frames_.clear();
    accumulated_points_ = 0U;
}

} // namespace dart_vision::lidar
