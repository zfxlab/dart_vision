#ifndef DART_AIMING_AIMING_CORE_HPP
#define DART_AIMING_AIMING_CORE_HPP

#include <array>
#include <optional>

namespace dart_vision::aiming {

/// 输出参考坐标系中的瞄准结果，角度单位为弧度，距离单位为米。
struct Aim {
    double yaw_error_rad{};
    double distance_m{};
};

/// 输入坐标为 x 前、y 左、z 上；距离只计算 XY 水平分量，偏转角向左为正。
[[nodiscard]] std::optional<Aim> solve(double x, double y, double z) noexcept;

/// 分别以几何 yaw 和距离为输入，用二次函数直接生成输出值。
class QuadraticAimModel {
  public:
    using Coefficients = std::array<double, 3>;

    QuadraticAimModel(Coefficients yaw_coefficients, Coefficients distance_coefficients);
    [[nodiscard]] std::optional<Aim> apply(const Aim& input) const noexcept;

  private:
    Coefficients yaw_coefficients_, distance_coefficients_;
};

/// 加入控制器给出的飞镖固定偏角。
[[nodiscard]] std::optional<Aim> applyDartOffset(const Aim& input, double offset_rad) noexcept;

} // namespace dart_vision::aiming
#endif // DART_AIMING_AIMING_CORE_HPP
