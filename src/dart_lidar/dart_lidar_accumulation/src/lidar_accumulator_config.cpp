#include "dart_lidar_accumulation/lidar_accumulator_config.hpp"

#include <cmath>

namespace dart_vision::lidar {

std::string StabilityConfig::validationError() const {
    if (!std::isfinite(hold_s) || hold_s <= 0.0) {
        return "stability.hold_s must be finite and positive";
    }
    if (!std::isfinite(yaw_span_rad) || yaw_span_rad <= 0.0) {
        return "stability.yaw_span_rad must be finite and positive";
    }
    if (!std::isfinite(abort_yaw_deviation_rad) || abort_yaw_deviation_rad <= 0.0) {
        return "stability.abort_yaw_deviation_rad must be finite and positive";
    }
    if (!std::isfinite(controller_timeout_s) || controller_timeout_s <= hold_s) {
        return "stability.controller_timeout_s must be greater than stability.hold_s";
    }
    if (!std::isfinite(post_stable_delay_s) || post_stable_delay_s < 0.05) {
        return "stability.post_stable_delay_s must be finite and at least 0.05";
    }
    if (!std::isfinite(measurement_deadline_s) || measurement_deadline_s <= 0.0) {
        return "stability.measurement_deadline_s must be finite and positive";
    }
    if (!std::isfinite(localization_reserve_s) || localization_reserve_s < 0.0 ||
        localization_reserve_s >= measurement_deadline_s) {
        return "stability.localization_reserve_s must be non-negative and less than "
               "stability.measurement_deadline_s";
    }
    return {};
}

std::string TransformConfig::validationError() const {
    if (!crop_box_min_m.allFinite()) {
        return "transform.crop_box_min_m must contain finite values";
    }
    if (!crop_box_max_m.allFinite()) {
        return "transform.crop_box_max_m must contain finite values";
    }
    if ((crop_box_min_m.array() > crop_box_max_m.array()).any()) {
        return "transform.crop_box_min_m must not exceed transform.crop_box_max_m";
    }
    return {};
}

std::string AccumulationConfig::validationError() const {
    if (!std::isfinite(duration_s) || duration_s <= 0.0) {
        return "accumulation.duration_s must be finite and positive";
    }
    if (min_frames == 0U) {
        return "accumulation.min_frames must be positive";
    }
    if (max_frames < min_frames) {
        return "accumulation.max_frames must be at least accumulation.min_frames";
    }
    if (max_points == 0U) {
        return "accumulation.max_points must be positive";
    }
    if (!std::isfinite(output_voxel_leaf_size_m) || output_voxel_leaf_size_m <= 0.0) {
        return "accumulation.output_voxel_leaf_size_m must be finite and positive";
    }
    return {};
}

std::string LidarAccumulatorConfig::validationError() const {
    if (mode_name != "triggered_online" && mode_name != "bag_offline") {
        return "mode must be 'triggered_online' or 'bag_offline'";
    }
    if (input_topic.empty()) {
        return "input_topic must not be empty";
    }
    if (output_topic.empty()) {
        return "output_topic must not be empty";
    }
    if (accumulation_frame.empty()) {
        return "accumulation_frame must not be empty";
    }
    if (mode == RuntimeMode::kTriggeredOnline && controller_state_topic.empty()) {
        return "controller_state_topic must not be empty in triggered_online mode";
    }
    if (const std::string error = transform.validationError(); !error.empty()) {
        return error;
    }
    if (const std::string error = accumulation.validationError(); !error.empty()) {
        return error;
    }
    if (mode == RuntimeMode::kTriggeredOnline) {
        return stability.validationError();
    }
    return {};
}

} // namespace dart_vision::lidar
