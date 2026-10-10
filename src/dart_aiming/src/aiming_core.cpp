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

std::optional<Aim> solve(double x, double y, double z) noexcept {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || x <= 0.0)
        return std::nullopt;
    const double distance = std::hypot(x, y);
    if (!std::isfinite(distance))
        return std::nullopt;
    return Aim{wrapAngle(std::atan2(y, x)), distance};
}

std::optional<Aim> applyDartOffset(const Aim& input, const double offset_rad) noexcept {
    if (!std::isfinite(input.yaw_error_rad) || !std::isfinite(input.distance_m) ||
        input.distance_m <= 0.0 || !std::isfinite(offset_rad))
        return std::nullopt;
    return Aim{wrapAngle(input.yaw_error_rad + offset_rad), input.distance_m};
}

DistanceMovingAverage::DistanceMovingAverage(int window_frames) {
    if (window_frames < 1)
        throw std::invalid_argument("Distance average window must be positive");
    window_frames_ = static_cast<std::size_t>(window_frames);
}

void DistanceMovingAverage::reset() noexcept {
    samples_.clear();
}

std::optional<double> DistanceMovingAverage::update(double distance_m) {
    if (!std::isfinite(distance_m) || distance_m <= 0.0) {
        reset();
        return std::nullopt;
    }
    samples_.push_back(distance_m);
    if (samples_.size() > window_frames_)
        samples_.pop_front();
    if (samples_.size() < window_frames_)
        return std::nullopt;
    // 按窗口重新求和，避免滚动累计在长期运行中产生误差漂移。
    double mean = 0.0;
    for (const double sample : samples_)
        mean += sample / static_cast<double>(window_frames_);
    return mean;
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
