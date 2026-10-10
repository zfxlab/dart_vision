#include "dart_camera/bearing_kalman_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dart_vision::camera {

bool BearingKalmanFilterConfig::isConfigValid() const noexcept {
    return std::isfinite(process_noise_rad2_per_s) && process_noise_rad2_per_s >= 0.0 &&
           std::isfinite(measurement_noise_rad2) && measurement_noise_rad2 > 0.0 &&
           std::isfinite(initial_uncertainty_rad2) && initial_uncertainty_rad2 > 0.0 &&
           std::isfinite(max_time_step_s) && max_time_step_s > 0.0;
}

BearingKalmanFilter::BearingKalmanFilter(const BearingKalmanFilterConfig& config)
    : config_(config) {
    if (!config_.isConfigValid())
        throw std::invalid_argument("Invalid bearing Kalman filter configuration");
}

cv::Vec3d BearingKalmanFilter::update(const cv::Vec3d& measured_bearing, const double time_step_s) {
    const double norm = cv::norm(measured_bearing);
    if (!std::isfinite(norm) || norm <= 0.0 || !std::isfinite(measured_bearing[0]) ||
        !std::isfinite(measured_bearing[1]) || !std::isfinite(measured_bearing[2]) ||
        measured_bearing[2] <= 0.0)
        throw std::invalid_argument("Measured bearing must be finite and point forward");
    if (!std::isfinite(time_step_s) || time_step_s <= 0.0)
        throw std::invalid_argument("Kalman filter time step must be finite and positive");

    const cv::Vec3d bearing = measured_bearing / norm;
    const double measured_yaw = std::atan2(bearing[0], bearing[2]);
    const double measured_pitch = std::atan2(bearing[1], std::hypot(bearing[0], bearing[2]));
    const double filtered_yaw = updateAngle(yaw_, measured_yaw, time_step_s);
    const double filtered_pitch = updateAngle(pitch_, measured_pitch, time_step_s);

    const double cos_pitch = std::cos(filtered_pitch);
    return {std::sin(filtered_yaw) * cos_pitch, std::sin(filtered_pitch),
            std::cos(filtered_yaw) * cos_pitch};
}

void BearingKalmanFilter::reset() noexcept {
    yaw_ = AngleState{};
    pitch_ = AngleState{};
}

double BearingKalmanFilter::updateAngle(AngleState& state, const double measurement_rad,
                                        const double time_step_s) const {
    const double measurement = normalizeAngle(measurement_rad);
    if (!state.initialized) {
        state.estimate_rad = measurement;
        state.uncertainty_rad2 = config_.initial_uncertainty_rad2;
        state.initialized = true;
        return state.estimate_rad;
    }

    const double effective_time_step = std::min(time_step_s, config_.max_time_step_s);
    const double predicted_uncertainty =
        state.uncertainty_rad2 + config_.process_noise_rad2_per_s * effective_time_step;
    const double gain =
        predicted_uncertainty / (predicted_uncertainty + config_.measurement_noise_rad2);
    const double innovation = normalizeAngle(measurement - state.estimate_rad);
    state.estimate_rad = normalizeAngle(state.estimate_rad + gain * innovation);
    state.uncertainty_rad2 = (1.0 - gain) * predicted_uncertainty;
    return state.estimate_rad;
}

double BearingKalmanFilter::normalizeAngle(const double angle_rad) noexcept {
    constexpr double kTwoPi = 6.28318530717958647692;
    return std::remainder(angle_rad, kTwoPi);
}

} // namespace dart_vision::camera
