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
        case StereoTriangulationRejection::distance_too_large:
            return "distance_too_large";
        case StereoTriangulationRejection::behind_reference_frame:
            return "behind_reference_frame";
    }
    return "unknown";
}

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
                                const cv::Vec3d& right_bearing,
                                StereoTriangulationDiagnostics* diagnostics) const noexcept {
    StereoTriangulationDiagnostics local_diagnostics;
    auto& diagnostic = diagnostics ? *diagnostics : local_diagnostics;
    diagnostic = StereoTriangulationDiagnostics{};
    const auto reject = [&](const StereoTriangulationRejection reason)
        -> std::optional<StereoTriangulationResult> {
        diagnostic.rejection = reason;
        return std::nullopt;
    };

    if (!finiteVector(left_origin) || !finiteVector(right_origin) || !finiteVector(left_bearing) ||
        !finiteVector(right_bearing))
        return reject(StereoTriangulationRejection::non_finite_input);

    diagnostic.baseline_m = cv::norm(left_origin - right_origin);
    if (!std::isfinite(diagnostic.baseline_m) || diagnostic.baseline_m <= 1e-12)
        return reject(StereoTriangulationRejection::degenerate_baseline);

    const double left_norm = cv::norm(left_bearing);
    const double right_norm = cv::norm(right_bearing);
    if (!std::isfinite(left_norm) || !std::isfinite(right_norm) || left_norm <= 0.0 ||
        right_norm <= 0.0)
        return reject(StereoTriangulationRejection::invalid_bearing);

    const cv::Vec3d left_direction = left_bearing / left_norm;
    const cv::Vec3d right_direction = right_bearing / right_norm;
    const double dot = std::clamp(left_direction.dot(right_direction), -1.0, 1.0);
    const double ray_angle_rad = std::acos(dot);
    diagnostic.ray_angle_deg = ray_angle_rad * 180.0 / kPi;
    if (ray_angle_rad < config_.min_ray_angle_deg * kPi / 180.0)
        return reject(StereoTriangulationRejection::ray_angle_too_small);

    // 最小化两条射线上点的间距，两条射线均从各自光心向前延伸。
    const cv::Vec3d w0 = left_origin - right_origin;
    const double d = left_direction.dot(w0);
    const double e = right_direction.dot(w0);
    const double denominator = 1.0 - dot * dot;
    if (!std::isfinite(denominator) || denominator <= 1e-12)
        return reject(StereoTriangulationRejection::near_parallel_rays);
    diagnostic.left_distance_m = (dot * e - d) / denominator;
    diagnostic.right_distance_m = (e - dot * d) / denominator;
    if (!std::isfinite(diagnostic.left_distance_m) ||
        !std::isfinite(diagnostic.right_distance_m))
        return reject(StereoTriangulationRejection::non_finite_depth);
    if (diagnostic.left_distance_m <= 0.0 || diagnostic.right_distance_m <= 0.0)
        return reject(StereoTriangulationRejection::behind_camera);
    if (diagnostic.left_distance_m < config_.min_depth_m ||
        diagnostic.right_distance_m < config_.min_depth_m)
        return reject(StereoTriangulationRejection::depth_below_minimum);

    const cv::Vec3d left_point =
        left_origin + diagnostic.left_distance_m * left_direction;
    const cv::Vec3d right_point =
        right_origin + diagnostic.right_distance_m * right_direction;
    diagnostic.ray_gap_m = cv::norm(left_point - right_point);
    const cv::Vec3d midpoint = 0.5 * (left_point + right_point);
    diagnostic.distance_m = cv::norm(midpoint);
    diagnostic.midpoint_x_m = midpoint[0];
    if (!finiteVector(midpoint) || !std::isfinite(diagnostic.ray_gap_m) ||
        !std::isfinite(diagnostic.distance_m))
        return reject(StereoTriangulationRejection::non_finite_result);
    if (diagnostic.ray_gap_m > config_.max_ray_gap_m)
        return reject(StereoTriangulationRejection::ray_gap_too_large);
    if (diagnostic.distance_m > config_.max_distance_m)
        return reject(StereoTriangulationRejection::distance_too_large);
    if (diagnostic.midpoint_x_m <= 0.0)
        return reject(StereoTriangulationRejection::behind_reference_frame);

    return StereoTriangulationResult{midpoint,
                                     diagnostic.distance_m,
                                     diagnostic.ray_gap_m,
                                     diagnostic.left_distance_m,
                                     diagnostic.right_distance_m};
}

} // namespace dart_vision::stereo
