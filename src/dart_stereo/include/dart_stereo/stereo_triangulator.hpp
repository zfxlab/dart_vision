#ifndef DART_STEREO_STEREO_TRIANGULATOR_HPP
#define DART_STEREO_STEREO_TRIANGULATOR_HPP

#include <limits>
#include <opencv2/core.hpp>
#include <optional>

namespace dart_vision::stereo {

/// 双目三角测量的有效性判定阈值。
struct StereoTriangulatorConfig {
    /// 两条三维视线的最小交会锐角，同时排除接近平行与反平行。
    double min_ray_angle_deg{0.05};
    /// 从各自光心沿原始三维单位视线前进的最小距离。
    double min_depth_m{0.1};
    /// 估计点到任一相机光心的最大空间距离。
    double max_camera_range_m{50.0};
    /// 两条空间视线最近点的最大间距。
    double max_ray_gap_m{0.1};

    [[nodiscard]] bool isConfigValid() const noexcept;
};

struct StereoTriangulationResult {
    /// 两条三维视线最近点的中点，单位为米。
    cv::Vec3d position_m{};
    /// 参考原点到目标的水平距离。
    double distance_m{};
    double ray_gap_m{};
    double left_ray_distance_m{};
    double right_ray_distance_m{};
};

enum class StereoTriangulationRejection {
    none,
    non_finite_input,
    degenerate_baseline,
    invalid_bearing,
    ray_angle_too_small,
    near_parallel_rays,
    non_finite_depth,
    behind_camera,
    depth_below_minimum,
    non_finite_result,
    ray_gap_too_large,
    camera_range_too_large,
};

[[nodiscard]] const char*
stereoTriangulationRejectionName(StereoTriangulationRejection rejection) noexcept;

/// 失败时也保留已经计算出的几何中间量，未计算的字段为 NaN。
struct StereoTriangulationDiagnostics {
    StereoTriangulationRejection rejection{StereoTriangulationRejection::none};
    double baseline_m{std::numeric_limits<double>::quiet_NaN()};
    double ray_angle_deg{std::numeric_limits<double>::quiet_NaN()};
    double left_distance_m{std::numeric_limits<double>::quiet_NaN()};
    double right_distance_m{std::numeric_limits<double>::quiet_NaN()};
    double ray_gap_m{std::numeric_limits<double>::quiet_NaN()};
    double distance_m{std::numeric_limits<double>::quiet_NaN()};
    double camera_range_m{std::numeric_limits<double>::quiet_NaN()};
    double midpoint_x_m{std::numeric_limits<double>::quiet_NaN()};
};

/** 求两条三维视线最近点的中点；不依赖参考坐标系的朝向。 */
class StereoTriangulator {
  public:
    explicit StereoTriangulator(const StereoTriangulatorConfig& config);

    /// 两个光心位置及视线方向必须位于同一正交坐标系。
    [[nodiscard]] std::optional<StereoTriangulationResult>
    triangulate(const cv::Vec3d& left_origin, const cv::Vec3d& left_bearing,
                const cv::Vec3d& right_origin, const cv::Vec3d& right_bearing,
                StereoTriangulationDiagnostics* diagnostics = nullptr) const noexcept;

  private:
    StereoTriangulatorConfig config_;
};

} // namespace dart_vision::stereo

#endif // DART_STEREO_STEREO_TRIANGULATOR_HPP
