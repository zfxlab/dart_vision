#include "dart_aiming/aiming_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dart_vision::aiming {
namespace {
double wrapAngle(double value) noexcept {
    return std::atan2(std::sin(value), std::cos(value));
}
} // namespace

std::optional<Aim> solve(double x, double y, double z, double offset_rad) noexcept {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(offset_rad) ||
        x <= 0.0)
        return std::nullopt;
    const double distance = std::hypot(x, y, z);
    if (!std::isfinite(distance))
        return std::nullopt;
    return Aim{wrapAngle(-std::atan2(y, x) + offset_rad), distance};
}

Stability::Stability(int frames, double yaw_step, double distance_step)
    : frames_(frames), yaw_step_(yaw_step), distance_step_(distance_step) {
    if (frames < 1 || !std::isfinite(yaw_step) || yaw_step <= 0.0 ||
        !std::isfinite(distance_step) || distance_step <= 0.0)
        throw std::invalid_argument("Invalid stability thresholds");
}

void Stability::reset() noexcept {
    previous_.reset();
    count_ = 0;
}

bool Stability::update(const Aim& aim) noexcept {
    if (previous_ &&
        std::abs(wrapAngle(aim.yaw_error_rad - previous_->yaw_error_rad)) <= yaw_step_ &&
        std::abs(aim.distance_m - previous_->distance_m) <= distance_step_) {
        count_ = std::min(count_ + 1, frames_);
    } else {
        count_ = 1;
    }
    previous_ = aim;
    return count_ >= frames_;
}
} // namespace dart_vision::aiming
