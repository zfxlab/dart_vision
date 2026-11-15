#include "dart_camera/pixel_center_ema.hpp"

#include <cmath>
#include <stdexcept>

namespace dart_vision::camera {
bool PixelCenterEmaConfig::isValid() const noexcept {
    return std::isfinite(tau_x_s) && tau_x_s > 0.0 && std::isfinite(tau_y_s) && tau_y_s > 0.0 &&
           std::isfinite(reset_gap_s) && reset_gap_s > 0.0;
}

PixelCenterEma::PixelCenterEma(const PixelCenterEmaConfig& config) {
    configure(config);
}

void PixelCenterEma::configure(const PixelCenterEmaConfig& config) {
    if (!config.isValid())
        throw std::invalid_argument("EMA time constants and reset gap must be finite and positive");
    const bool changed = config.enabled != config_.enabled || config.tau_x_s != config_.tau_x_s ||
                         config.tau_y_s != config_.tau_y_s ||
                         config.reset_gap_s != config_.reset_gap_s;
    config_ = config;
    if (changed)
        reset();
}

void PixelCenterEma::reset() noexcept {
    stamp_ns_.reset();
}

const PixelCenterEmaConfig& PixelCenterEma::config() const noexcept {
    return config_;
}

cv::Point2f PixelCenterEma::update(const cv::Point2f& center, const std::int64_t stamp_ns) {
    if (!std::isfinite(center.x) || !std::isfinite(center.y) || stamp_ns < 0) {
        reset();
        throw std::invalid_argument("Invalid EMA observation or timestamp");
    }
    if (!config_.enabled) {
        reset();
        return center;
    }
    // Subtract integer stamps first to preserve small dt at epoch timestamps.
    const double dt = stamp_ns_ && stamp_ns > *stamp_ns_
                          ? static_cast<double>(stamp_ns - *stamp_ns_) * 1e-9
                          : 0.0;
    if (!stamp_ns_ || dt <= 0.0 || dt > config_.reset_gap_s) {
        center_ = center;
    } else {
        center_.x += -std::expm1(-dt / config_.tau_x_s) * (center.x - center_.x);
        center_.y += -std::expm1(-dt / config_.tau_y_s) * (center.y - center_.y);
    }
    stamp_ns_ = stamp_ns;
    return cv::Point2f(center_);
}
} // namespace dart_vision::camera
