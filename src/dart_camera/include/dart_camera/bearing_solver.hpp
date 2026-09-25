#ifndef DART_CAMERA_BEARING_SOLVER_HPP
#define DART_CAMERA_BEARING_SOLVER_HPP

#include <array>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace dart_vision::camera {

/// 像素视线计算所需的相机标定参数。
struct BearingSolverConfig {
    std::array<double, 9> camera_matrix{};       ///< 按行存储的 3x3 相机内参矩阵。
    std::vector<double> distortion_coefficients; ///< OpenCV 格式的镜头畸变系数。

    [[nodiscard]] bool isConfigValid() const noexcept;
};

/**
 * @brief 将目标像素中心转换为相机光学坐标系中的单位视线。
 *
 * 输出遵循 ROS 光学坐标系约定：+x 向图像右侧、+y 向图像下方、+z 向相机前方。
 * 本类只恢复方向，不根据目标图像尺寸估计距离。
 */
class BearingSolver {
public:
    explicit BearingSolver(const BearingSolverConfig& config);

    /**
     * @param center_px 目标中心，单位为像素。
     * @return 单位视线；输入无效或去畸变失败时返回 std::nullopt。
     */
    [[nodiscard]] std::optional<cv::Vec3d>
    calculateUnitBearing(const cv::Point2f& center_px) const noexcept;

private:
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;
};

} // namespace dart_vision::camera

#endif // DART_CAMERA_BEARING_SOLVER_HPP
