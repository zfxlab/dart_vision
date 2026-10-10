#ifndef DART_CAMERA_BEARING_KALMAN_FILTER_HPP
#define DART_CAMERA_BEARING_KALMAN_FILTER_HPP

#include <opencv2/core.hpp>

namespace dart_vision::camera {

/// 单目单位射线角度 Kalman 滤波器参数。
struct BearingKalmanFilterConfig {
    double process_noise_rad2_per_s{0.01};
    double measurement_noise_rad2{0.0025};
    double initial_uncertainty_rad2{0.01};
    double max_time_step_s{0.1};

    [[nodiscard]] bool isConfigValid() const noexcept;
};

/// 分别滤波光学坐标系中的水平角和垂直角，并重建单位射线。
class BearingKalmanFilter {
  public:
    explicit BearingKalmanFilter(const BearingKalmanFilterConfig& config);

    [[nodiscard]] cv::Vec3d update(const cv::Vec3d& measured_bearing, double time_step_s);
    void reset() noexcept;

  private:
    struct AngleState {
        double estimate_rad{};
        double uncertainty_rad2{};
        bool initialized{};
    };

    BearingKalmanFilterConfig config_;
    AngleState yaw_, pitch_;

    [[nodiscard]] double updateAngle(AngleState& state, double measurement_rad,
                                     double time_step_s) const;
    [[nodiscard]] static double normalizeAngle(double angle_rad) noexcept;
};

} // namespace dart_vision::camera

#endif // DART_CAMERA_BEARING_KALMAN_FILTER_HPP
