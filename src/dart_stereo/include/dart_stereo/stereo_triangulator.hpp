#ifndef DART_STEREO_STEREO_TRIANGULATOR_HPP
#define DART_STEREO_STEREO_TRIANGULATOR_HPP

#include <opencv2/core.hpp>
#include <optional>

namespace dart_vision::stereo {

/// 双目三角测量的有效性判定阈值。
struct StereoTriangulatorConfig {
    double min_ray_angle_deg{0.05};
    double min_depth_m{0.1};
    double max_distance_m{50.0};
    double max_ray_gap_m{0.1};

    [[nodiscard]] bool isConfigValid() const noexcept;
};

struct StereoTriangulationResult {
    /// 固定板中心坐标系中的三维位置，单位为米。
    cv::Vec3d position_m{};
    double distance_m{};
    double ray_gap_m{};
    double left_ray_distance_m{};
    double right_ray_distance_m{};
};

/** 在固定板中心坐标系中，取两条视线最近点连线的中点作为目标位置。 */
class StereoTriangulator {
public:
    explicit StereoTriangulator(const StereoTriangulatorConfig& config);

    /// 两个光心位置及视线方向必须位于同一固定板中心坐标系（x 前、y 左、z 上）。
    [[nodiscard]] std::optional<StereoTriangulationResult>
    triangulate(const cv::Vec3d& left_origin,
                const cv::Vec3d& left_bearing,
                const cv::Vec3d& right_origin,
                const cv::Vec3d& right_bearing) const noexcept;

private:
    StereoTriangulatorConfig config_;
};

} // namespace dart_vision::stereo

#endif // DART_STEREO_STEREO_TRIANGULATOR_HPP
