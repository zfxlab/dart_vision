#ifndef DART_STEREO_STEREO_TRIANGULATOR_HPP
#define DART_STEREO_STEREO_TRIANGULATOR_HPP

#include <limits>
#include <optional>

namespace dart_vision::stereo {

/// 平行双目视差计算及质量判定参数。
struct StereoTriangulatorConfig {
    double baseline_m{0.3};
    double min_normalized_disparity{1e-6};
    double min_distance_m{0.5};
    double max_distance_m{30.0};
    double max_vertical_residual{0.002};

    [[nodiscard]] bool isConfigValid() const noexcept;
};

struct StereoTriangulationResult {
    double forward_m{};
    double horizontal_right_m{};
    double horizontal_distance_m{};
    double yaw_rad{};
};

enum class StereoTriangulationRejection {
    none,
    non_finite_input,
    non_positive_disparity,
    vertical_residual_too_large,
    non_finite_result,
    distance_out_of_range,
};

[[nodiscard]] const char*
stereoTriangulationRejectionName(StereoTriangulationRejection rejection) noexcept;

struct StereoTriangulationDiagnostics {
    StereoTriangulationRejection rejection{StereoTriangulationRejection::none};
    double normalized_disparity{std::numeric_limits<double>::quiet_NaN()};
    double vertical_residual{std::numeric_limits<double>::quiet_NaN()};
    double forward_m{std::numeric_limits<double>::quiet_NaN()};
    double horizontal_right_m{std::numeric_limits<double>::quiet_NaN()};
    double horizontal_distance_m{std::numeric_limits<double>::quiet_NaN()};
    double yaw_rad{std::numeric_limits<double>::quiet_NaN()};
};

/** 使用去主点、归一化后的左右像素坐标计算水平位置。 */
class StereoTriangulator {
public:
    explicit StereoTriangulator(const StereoTriangulatorConfig& config);

    [[nodiscard]] std::optional<StereoTriangulationResult>
    triangulate(double left_x,
                double left_y,
                double right_x,
                double right_y,
                StereoTriangulationDiagnostics* diagnostics = nullptr) const noexcept;

private:
    StereoTriangulatorConfig config_;
};

} // namespace dart_vision::stereo

#endif // DART_STEREO_STEREO_TRIANGULATOR_HPP
