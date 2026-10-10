#include "dart_aiming/aiming_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dart_vision::aiming {
namespace {
double wrapAngle(double value) noexcept {
    return std::atan2(std::sin(value), std::cos(value));
}

bool finiteCoefficients(const QuadraticAimModel::Coefficients& coefficients) noexcept {
    return std::all_of(coefficients.begin(), coefficients.end(),
                       [](const double value) { return std::isfinite(value); });
}

double evaluateQuadratic(const QuadraticAimModel::Coefficients& coefficients,
                         const double input) noexcept {
    return (coefficients[0] * input + coefficients[1]) * input + coefficients[2];
}
} // namespace

std::optional<Aim> solve(double x, double y, double z) noexcept {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || x <= 0.0)
        return std::nullopt;
    const double distance = std::hypot(x, y);
    if (!std::isfinite(distance))
        return std::nullopt;
    return Aim{wrapAngle(std::atan2(y, x)), distance};
}

QuadraticAimModel::QuadraticAimModel(Coefficients yaw_coefficients,
                                     Coefficients distance_coefficients)
    : yaw_coefficients_(yaw_coefficients), distance_coefficients_(distance_coefficients) {
    if (!finiteCoefficients(yaw_coefficients_) || !finiteCoefficients(distance_coefficients_))
        throw std::invalid_argument("Quadratic aim model coefficients must be finite");
}

std::optional<Aim> QuadraticAimModel::apply(const Aim& input) const noexcept {
    if (!std::isfinite(input.yaw_error_rad) || !std::isfinite(input.distance_m) ||
        input.distance_m <= 0.0)
        return std::nullopt;
    const Aim output{evaluateQuadratic(yaw_coefficients_, input.yaw_error_rad),
                     evaluateQuadratic(distance_coefficients_, input.distance_m)};
    if (!std::isfinite(output.yaw_error_rad) || !std::isfinite(output.distance_m) ||
        output.distance_m <= 0.0)
        return std::nullopt;
    return output;
}

std::optional<Aim> applyDartOffset(const Aim& input, const double offset_rad) noexcept {
    if (!std::isfinite(input.yaw_error_rad) || !std::isfinite(input.distance_m) ||
        input.distance_m <= 0.0 || !std::isfinite(offset_rad))
        return std::nullopt;
    return Aim{wrapAngle(input.yaw_error_rad + offset_rad), input.distance_m};
}

} // namespace dart_vision::aiming
