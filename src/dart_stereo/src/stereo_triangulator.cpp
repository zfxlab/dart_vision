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
    case StereoTriangulationRejection::ray_gap_too_large:
        return "ray_gap_too_large";
    case StereoTriangulationRejection::camera_range_too_large:
        return "camera_range_too_large";
    }
    return "unknown";
}

bool StereoTriangulatorConfig::isConfigValid() const noexcept {
    return std::isfinite(min_ray_angle_deg) && min_ray_angle_deg > 0.0 &&
           min_ray_angle_deg < 90.0 && std::isfinite(min_depth_m) && min_depth_m >= 0.0 &&
           std::isfinite(max_camera_range_m) && max_camera_range_m > 0.0 &&
           std::isfinite(max_ray_gap_m) && max_ray_gap_m >= 0.0;
}

StereoTriangulator::StereoTriangulator(const StereoTriangulatorConfig& config) : config_(config) {
    if (!config_.isConfigValid()) {
        throw std::invalid_argument("Invalid stereo triangulator configuration");
    }
}

std::optional<StereoTriangulationResult>
StereoTriangulator::triangulate(const cv::Vec3d& top_origin, const cv::Vec3d& top_bearing,
                                const cv::Vec3d& bottom_origin, const cv::Vec3d& bottom_bearing,
                                StereoTriangulationDiagnostics* diagnostics) const noexcept {
    StereoTriangulationDiagnostics local_diagnostics;
    auto& diagnostic = diagnostics ? *diagnostics : local_diagnostics;
    diagnostic = StereoTriangulationDiagnostics{};
    const auto reject =
        [&](const StereoTriangulationRejection reason) -> std::optional<StereoTriangulationResult> {
        diagnostic.rejection = reason;
        return std::nullopt;
    };

    if (!finiteVector(top_origin) || !finiteVector(bottom_origin) || !finiteVector(top_bearing) ||
        !finiteVector(bottom_bearing))
        return reject(StereoTriangulationRejection::non_finite_input);

    diagnostic.baseline_m = cv::norm(top_origin - bottom_origin);
    if (!std::isfinite(diagnostic.baseline_m) || diagnostic.baseline_m <= 1e-12)
        return reject(StereoTriangulationRejection::degenerate_baseline);

    const double top_norm = cv::norm(top_bearing);
    const double bottom_norm = cv::norm(bottom_bearing);
    if (!std::isfinite(top_norm) || !std::isfinite(bottom_norm) || top_norm <= 0.0 ||
        bottom_norm <= 0.0)
        return reject(StereoTriangulationRejection::invalid_bearing);

    const cv::Vec3d top_direction = top_bearing / top_norm;
    const cv::Vec3d bottom_direction = bottom_bearing / bottom_norm;
    // 三维交会锐角：平行与反平行均退化。叉积避免 1-dot^2 的相消误差。
    const cv::Vec3d cross = top_direction.cross(bottom_direction);
    const double denominator = cross.dot(cross);
    const double dot = std::clamp(top_direction.dot(bottom_direction), -1.0, 1.0);
    const double ray_angle_rad = std::atan2(std::sqrt(denominator), std::abs(dot));
    diagnostic.ray_angle_deg = ray_angle_rad * 180.0 / kPi;
    if (denominator <= 1e-24)
        return reject(StereoTriangulationRejection::near_parallel_rays);
    if (ray_angle_rad < config_.min_ray_angle_deg * kPi / 180.0)
        return reject(StereoTriangulationRejection::ray_angle_too_small);

    // 最小化 |o_T + s*d_T - o_B - t*d_B|^2；s/t 为沿单位射线的距离。
    const cv::Vec3d offset = bottom_origin - top_origin;
    diagnostic.top_distance_m = offset.cross(bottom_direction).dot(cross) / denominator;
    diagnostic.bottom_distance_m = offset.cross(top_direction).dot(cross) / denominator;
    if (!std::isfinite(diagnostic.top_distance_m) || !std::isfinite(diagnostic.bottom_distance_m))
        return reject(StereoTriangulationRejection::non_finite_depth);
    if (diagnostic.top_distance_m <= 0.0 || diagnostic.bottom_distance_m <= 0.0)
        return reject(StereoTriangulationRejection::behind_camera);
    if (diagnostic.top_distance_m < config_.min_depth_m ||
        diagnostic.bottom_distance_m < config_.min_depth_m)
        return reject(StereoTriangulationRejection::depth_below_minimum);

    const cv::Vec3d top_point = top_origin + diagnostic.top_distance_m * top_direction;
    const cv::Vec3d bottom_point = bottom_origin + diagnostic.bottom_distance_m * bottom_direction;
    diagnostic.ray_gap_m = cv::norm(top_point - bottom_point);
    const cv::Vec3d midpoint = 0.5 * (top_point + bottom_point);
    diagnostic.distance_m = std::hypot(midpoint[0], midpoint[1]);
    diagnostic.midpoint_x_m = midpoint[0];
    diagnostic.camera_range_m =
        std::max(cv::norm(midpoint - top_origin), cv::norm(midpoint - bottom_origin));
    if (!finiteVector(midpoint) || !std::isfinite(diagnostic.ray_gap_m) ||
        !std::isfinite(diagnostic.distance_m) || !std::isfinite(diagnostic.camera_range_m))
        return reject(StereoTriangulationRejection::non_finite_result);
    if (diagnostic.ray_gap_m > config_.max_ray_gap_m)
        return reject(StereoTriangulationRejection::ray_gap_too_large);
    if (diagnostic.camera_range_m > config_.max_camera_range_m)
        return reject(StereoTriangulationRejection::camera_range_too_large);

    return StereoTriangulationResult{midpoint, diagnostic.distance_m, diagnostic.ray_gap_m,
                                     diagnostic.top_distance_m, diagnostic.bottom_distance_m};
}

} // namespace dart_vision::stereo
