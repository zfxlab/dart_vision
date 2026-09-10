#include "dart_lidar_accumulation/lidar_accumulator_config.hpp"

#include <cmath>

namespace dart_vision::lidar {

std::string TransformConfig::validationError() const {
    if (!crop_box_min_m.allFinite() || !crop_box_max_m.allFinite()) {
        return "transform crop bounds must contain finite values";
    }
    if ((crop_box_min_m.array() > crop_box_max_m.array()).any()) {
        return "transform.crop_box_min_m must not exceed transform.crop_box_max_m";
    }
    return {};
}

std::string AccumulationConfig::validationError() const {
    if (!std::isfinite(window_duration_s) || window_duration_s <= 0.0) {
        return "accumulation.window_duration_s must be finite and positive";
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

std::string PublishConfig::validationError() const {
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
        return "publish.rate_hz must be finite and positive";
    }
    if (min_new_frames == 0U) {
        return "publish.min_new_frames must be positive";
    }
    return {};
}

std::string LidarAccumulatorConfig::validationError() const {
    if (input_topic.empty() || output_topic.empty() || accumulation_frame.empty()) {
        return "accumulator topics and frame must not be empty";
    }
    if (const std::string error = transform.validationError(); !error.empty()) {
        return error;
    }
    if (const std::string error = accumulation.validationError(); !error.empty()) {
        return error;
    }
    return publish.validationError();
}

} // namespace dart_vision::lidar
