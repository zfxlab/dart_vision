#include "dart_stereo/stereo_triangulator.hpp"

#include <cmath>
#include <stdexcept>

namespace dart_vision::stereo {

const char*
stereoTriangulationRejectionName(const StereoTriangulationRejection rejection) noexcept {
    switch (rejection) {
        case StereoTriangulationRejection::none:
            return "none";
        case StereoTriangulationRejection::non_finite_input:
            return "non_finite_input";
        case StereoTriangulationRejection::non_positive_disparity:
            return "non_positive_disparity";
        case StereoTriangulationRejection::vertical_residual_too_large:
            return "vertical_residual_too_large";
        case StereoTriangulationRejection::non_finite_result:
            return "non_finite_result";
        case StereoTriangulationRejection::distance_out_of_range:
            return "distance_out_of_range";
    }
    return "unknown";
}

bool StereoTriangulatorConfig::isConfigValid() const noexcept {
    return std::isfinite(baseline_m) && baseline_m > 0.0 &&
           std::isfinite(min_normalized_disparity) && min_normalized_disparity > 0.0 &&
           std::isfinite(min_distance_m) && min_distance_m >= 0.0 &&
           std::isfinite(max_distance_m) && max_distance_m > min_distance_m &&
           std::isfinite(max_vertical_residual) && max_vertical_residual >= 0.0;
}

StereoTriangulator::StereoTriangulator(const StereoTriangulatorConfig& config) : config_(config) {
    if (!config_.isConfigValid()) {
        throw std::invalid_argument("Invalid stereo disparity configuration");
    }
}

std::optional<StereoTriangulationResult>
StereoTriangulator::triangulate(const double left_x,
                                const double left_y,
                                const double right_x,
                                const double right_y,
                                StereoTriangulationDiagnostics* diagnostics) const noexcept {
    StereoTriangulationDiagnostics local_diagnostics;
    auto& diagnostic = diagnostics ? *diagnostics : local_diagnostics;
    diagnostic = StereoTriangulationDiagnostics{};
    const auto reject =
        [&](const StereoTriangulationRejection reason) -> std::optional<StereoTriangulationResult> {
        diagnostic.rejection = reason;
        return std::nullopt;
    };

    if (!std::isfinite(left_x) || !std::isfinite(left_y) || !std::isfinite(right_x) ||
        !std::isfinite(right_y)) {
        return reject(StereoTriangulationRejection::non_finite_input);
    }

    diagnostic.normalized_disparity = left_x - right_x;
    if (diagnostic.normalized_disparity <= config_.min_normalized_disparity) {
        return reject(StereoTriangulationRejection::non_positive_disparity);
    }

    diagnostic.vertical_residual = std::abs(left_y - right_y);
    if (diagnostic.vertical_residual > config_.max_vertical_residual) {
        return reject(StereoTriangulationRejection::vertical_residual_too_large);
    }

    diagnostic.forward_m = config_.baseline_m / diagnostic.normalized_disparity;
    diagnostic.horizontal_right_m = diagnostic.forward_m * 0.5 * (left_x + right_x);
    diagnostic.horizontal_distance_m =
        std::hypot(diagnostic.forward_m, diagnostic.horizontal_right_m);
    diagnostic.yaw_rad = std::atan2(diagnostic.horizontal_right_m, diagnostic.forward_m);
    if (!std::isfinite(diagnostic.forward_m) || !std::isfinite(diagnostic.horizontal_right_m) ||
        !std::isfinite(diagnostic.horizontal_distance_m) || !std::isfinite(diagnostic.yaw_rad)) {
        return reject(StereoTriangulationRejection::non_finite_result);
    }
    if (diagnostic.horizontal_distance_m < config_.min_distance_m ||
        diagnostic.horizontal_distance_m > config_.max_distance_m) {
        return reject(StereoTriangulationRejection::distance_out_of_range);
    }

    return StereoTriangulationResult{diagnostic.forward_m,
                                     diagnostic.horizontal_right_m,
                                     diagnostic.horizontal_distance_m,
                                     diagnostic.yaw_rad};
}

} // namespace dart_vision::stereo
