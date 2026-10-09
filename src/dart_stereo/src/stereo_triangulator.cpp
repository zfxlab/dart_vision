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

const char*
stereoTriangulationRejectionName(const StereoTriangulationRejection rejection) noexcept {
    switch (rejection) {
    case StereoTriangulationRejection::none:
        return "none";
    case StereoTriangulationRejection::non_finite_input:
        return "non_finite_input";
    case StereoTriangulationRejection::degenerate_baseline:
        return "degenerate_baseline";
    case StereoTriangulationRejection::invalid_bearing:
        return "invalid_bearing";
    case StereoTriangulationRejection::ray_angle_too_small:
        return "ray_angle_too_small";
    case StereoTriangulationRejection::near_parallel_rays:
        return "near_parallel_rays";
    case StereoTriangulationRejection::non_finite_depth:
        return "non_finite_depth";
    case StereoTriangulationRejection::behind_camera:
        return "behind_camera";
    case StereoTriangulationRejection::depth_below_minimum:
        return "depth_below_minimum";
    case StereoTriangulationRejection::non_finite_result:
        return "non_finite_result";
    case StereoTriangulationRejection::height_gap_too_large:
        return "height_gap_too_large";
    case StereoTriangulationRejection::distance_too_large:
        return "distance_too_large";
    case StereoTriangulationRejection::behind_reference_frame:
        return "behind_reference_frame";
    }
    return "unknown";
}

bool StereoTriangulatorConfig::isConfigValid() const noexcept {
    return std::isfinite(min_horizontal_ray_angle_deg) && min_horizontal_ray_angle_deg > 0.0 &&
           min_horizontal_ray_angle_deg < 90.0 && std::isfinite(min_depth_m) &&
           min_depth_m >= 0.0 && std::isfinite(max_distance_m) && max_distance_m > 0.0 &&
           std::isfinite(max_height_gap_m) && max_height_gap_m >= 0.0;
}

StereoTriangulator::StereoTriangulator(const StereoTriangulatorConfig& config) : config_(config) {
    if (!config_.isConfigValid()) {
        throw std::invalid_argument("Invalid stereo triangulator configuration");
    }
}

std::optional<StereoTriangulationResult>
StereoTriangulator::triangulate(const cv::Vec3d& left_origin, const cv::Vec3d& left_bearing,
                                const cv::Vec3d& right_origin, const cv::Vec3d& right_bearing,
                                StereoTriangulationDiagnostics* diagnostics) const noexcept {
    StereoTriangulationDiagnostics local_diagnostics;
    auto& diagnostic = diagnostics ? *diagnostics : local_diagnostics;
    diagnostic = StereoTriangulationDiagnostics{};
    const auto reject =
        [&](const StereoTriangulationRejection reason) -> std::optional<StereoTriangulationResult> {
        diagnostic.rejection = reason;
        return std::nullopt;
    };

    if (!finiteVector(left_origin) || !finiteVector(right_origin) || !finiteVector(left_bearing) ||
        !finiteVector(right_bearing))
        return reject(StereoTriangulationRejection::non_finite_input);

    diagnostic.baseline_m =
        std::hypot(left_origin[0] - right_origin[0], left_origin[1] - right_origin[1]);
    if (!std::isfinite(diagnostic.baseline_m) || diagnostic.baseline_m <= 1e-12)
        return reject(StereoTriangulationRejection::degenerate_baseline);

    const double left_norm = cv::norm(left_bearing);
    const double right_norm = cv::norm(right_bearing);
    if (!std::isfinite(left_norm) || !std::isfinite(right_norm) || left_norm <= 0.0 ||
        right_norm <= 0.0)
        return reject(StereoTriangulationRejection::invalid_bearing);

    const cv::Vec3d left_direction = left_bearing / left_norm;
    const cv::Vec3d right_direction = right_bearing / right_norm;
    // 保留三维单位方向，使求交参数仍表示沿原始空间射线的距离。
    const cv::Vec2d left_xy{left_direction[0], left_direction[1]};
    const cv::Vec2d right_xy{right_direction[0], right_direction[1]};
    const double left_xy_norm = cv::norm(left_xy);
    const double right_xy_norm = cv::norm(right_xy);
    if (left_xy_norm <= 1e-12 || right_xy_norm <= 1e-12)
        return reject(StereoTriangulationRejection::invalid_bearing);

    const auto cross = [](const cv::Vec2d& a, const cv::Vec2d& b) {
        return a[0] * b[1] - a[1] * b[0];
    };
    const double denominator = cross(left_xy, right_xy);
    const double normalized_cross = denominator / (left_xy_norm * right_xy_norm);
    const double dot =
        std::clamp(left_xy.dot(right_xy) / (left_xy_norm * right_xy_norm), -1.0, 1.0);
    const double ray_angle_rad = std::atan2(std::abs(normalized_cross), dot);
    diagnostic.ray_angle_deg = ray_angle_rad * 180.0 / kPi;
    if (ray_angle_rad < config_.min_horizontal_ray_angle_deg * kPi / 180.0)
        return reject(StereoTriangulationRejection::ray_angle_too_small);
    if (std::abs(normalized_cross) <= 1e-12)
        return reject(StereoTriangulationRejection::near_parallel_rays);

    // o_L + s*d_L.xy = o_R + t*d_R.xy。
    const cv::Vec2d offset{right_origin[0] - left_origin[0], right_origin[1] - left_origin[1]};
    diagnostic.left_distance_m = cross(offset, right_xy) / denominator;
    diagnostic.right_distance_m = cross(offset, left_xy) / denominator;
    if (!std::isfinite(diagnostic.left_distance_m) || !std::isfinite(diagnostic.right_distance_m))
        return reject(StereoTriangulationRejection::non_finite_depth);
    if (diagnostic.left_distance_m <= 0.0 || diagnostic.right_distance_m <= 0.0)
        return reject(StereoTriangulationRejection::behind_camera);
    if (diagnostic.left_distance_m < config_.min_depth_m ||
        diagnostic.right_distance_m < config_.min_depth_m)
        return reject(StereoTriangulationRejection::depth_below_minimum);

    const cv::Vec3d left_point = left_origin + diagnostic.left_distance_m * left_direction;
    const cv::Vec3d right_point = right_origin + diagnostic.right_distance_m * right_direction;
    diagnostic.height_gap_m = std::abs(left_point[2] - right_point[2]);
    const cv::Vec3d midpoint = 0.5 * (left_point + right_point);
    diagnostic.distance_m = std::hypot(midpoint[0], midpoint[1]);
    diagnostic.midpoint_x_m = midpoint[0];
    if (!finiteVector(midpoint) || !std::isfinite(diagnostic.height_gap_m) ||
        !std::isfinite(diagnostic.distance_m))
        return reject(StereoTriangulationRejection::non_finite_result);
    if (diagnostic.height_gap_m > config_.max_height_gap_m)
        return reject(StereoTriangulationRejection::height_gap_too_large);
    if (diagnostic.distance_m > config_.max_distance_m)
        return reject(StereoTriangulationRejection::distance_too_large);
    if (diagnostic.midpoint_x_m <= 0.0)
        return reject(StereoTriangulationRejection::behind_reference_frame);

    return StereoTriangulationResult{midpoint, diagnostic.distance_m, diagnostic.height_gap_m,
                                     diagnostic.left_distance_m, diagnostic.right_distance_m};
}

} // namespace dart_vision::stereo
