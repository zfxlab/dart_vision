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
    return Aim{wrapAngle(-std::atan2(y, x)), distance};
}

std::optional<Aim> applyFittedCorrection(const Aim& input) noexcept {
    // 在此处填写离线标定得到的拟合函数。input 是尚未加入 dart_offset_rad 的几何解；
    // corrected 再由 applyDartOffset() 加入固定偏角，生成最终输出。
    Aim corrected = input;

    // 示例（确定系数后替换，当前不要启用）：
    // corrected.yaw_error_rad += yaw_residual(input.distance_m);
    // corrected.distance_m += distance_residual(input.distance_m);

    if (!std::isfinite(corrected.yaw_error_rad) || !std::isfinite(corrected.distance_m) ||
        corrected.distance_m <= 0.0)
        return std::nullopt;
    corrected.yaw_error_rad = wrapAngle(corrected.yaw_error_rad);
    return corrected;
}

std::optional<Aim> applyDartOffset(const Aim& input, const double offset_rad) noexcept {
    if (!std::isfinite(input.yaw_error_rad) || !std::isfinite(input.distance_m) ||
        input.distance_m <= 0.0 || !std::isfinite(offset_rad))
        return std::nullopt;
    return Aim{wrapAngle(input.yaw_error_rad + offset_rad), input.distance_m};
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
