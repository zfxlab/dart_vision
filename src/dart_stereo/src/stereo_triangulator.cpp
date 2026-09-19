#include "dart_stereo/stereo_triangulator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dart_vision::stereo {
namespace {
constexpr double kPi = 3.14159265358979323846;

bool finiteVector(const cv::Vec3d& value) {
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}
} // namespace

bool StereoTriangulatorConfig::isConfigValid() const noexcept {
    return std::isfinite(min_ray_angle_deg) && min_ray_angle_deg > 0.0 &&
           min_ray_angle_deg < 90.0 && std::isfinite(min_depth_m) && min_depth_m >= 0.0 &&
           std::isfinite(max_distance_m) && max_distance_m > min_depth_m &&
           std::isfinite(max_ray_gap_m) && max_ray_gap_m >= 0.0;
}

StereoTriangulator::StereoTriangulator(const StereoTriangulatorConfig& config) : config_(config) {
    if (!config_.isConfigValid()) {
        throw std::invalid_argument("Invalid stereo triangulator configuration");
    }
}

std::optional<StereoTriangulationResult>
StereoTriangulator::triangulate(const cv::Vec3d& left_origin,
                                const cv::Vec3d& left_bearing,
                                const cv::Vec3d& right_origin,
                                const cv::Vec3d& right_bearing) const noexcept {
    if (!finiteVector(left_origin) || !finiteVector(right_origin) || !finiteVector(left_bearing) ||
        !finiteVector(right_bearing) || cv::norm(left_origin - right_origin) <= 1e-12)
        return std::nullopt;

    const double left_norm = cv::norm(left_bearing);
    const double right_norm = cv::norm(right_bearing);
    if (!std::isfinite(left_norm) || !std::isfinite(right_norm) || left_norm <= 0.0 ||
        right_norm <= 0.0)
        return std::nullopt;

    const cv::Vec3d left_direction = left_bearing / left_norm;
    const cv::Vec3d right_direction = right_bearing / right_norm;
    const double dot = std::clamp(left_direction.dot(right_direction), -1.0, 1.0);
    const double ray_angle_rad = std::acos(dot);
    if (ray_angle_rad < config_.min_ray_angle_deg * kPi / 180.0)
        return std::nullopt;

    // 最小化两条射线上点的间距，两条射线均从各自光心向前延伸。
    const cv::Vec3d w0 = left_origin - right_origin;
    const double d = left_direction.dot(w0);
    const double e = right_direction.dot(w0);
    const double denominator = 1.0 - dot * dot;
    if (!std::isfinite(denominator) || denominator <= 1e-12)
        return std::nullopt;
    const double left_distance = (dot * e - d) / denominator;
    const double right_distance = (e - dot * d) / denominator;
    if (!std::isfinite(left_distance) || !std::isfinite(right_distance) || left_distance <= 0.0 ||
        right_distance <= 0.0 || left_distance < config_.min_depth_m ||
        right_distance < config_.min_depth_m) {
        return std::nullopt;
    }

    const cv::Vec3d left_point = left_origin + left_distance * left_direction;
    const cv::Vec3d right_point = right_origin + right_distance * right_direction;
    const double ray_gap = cv::norm(left_point - right_point);
    const cv::Vec3d midpoint = 0.5 * (left_point + right_point);
    const double distance = cv::norm(midpoint);
    if (!finiteVector(midpoint) || !std::isfinite(ray_gap) || ray_gap > config_.max_ray_gap_m ||
        !std::isfinite(distance) || distance > config_.max_distance_m || midpoint[0] <= 0.0) {
        return std::nullopt;
    }

    return StereoTriangulationResult{midpoint, distance, ray_gap, left_distance, right_distance};
}

} // namespace dart_vision::stereo
