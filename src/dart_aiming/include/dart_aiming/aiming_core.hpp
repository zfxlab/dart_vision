#ifndef DART_AIMING_AIMING_CORE_HPP
#define DART_AIMING_AIMING_CORE_HPP

#include <optional>

namespace dart_vision::aiming {

/// 发射架坐标系中的瞄准结果，角度单位为弧度，距离单位为米。
struct Aim {
    double yaw_error_rad{};
    double distance_m{};
};

/// 输入坐标为 x 前、y 左、z 上；输出偏转角向右为正，补偿叠加一次。
[[nodiscard]] std::optional<Aim> solve(double x, double y, double z, double offset_rad) noexcept;

/// 使用相邻测量的变化量确认连续帧稳定性，不对结果取平均。
class Stability {
public:
    Stability(int frames, double yaw_step, double distance_step);
    void reset() noexcept;
    [[nodiscard]] bool update(const Aim& aim) noexcept;

private:
    int frames_;
    double yaw_step_, distance_step_;
    std::optional<Aim> previous_;
    int count_{};
};

} // namespace dart_vision::aiming
#endif // DART_AIMING_AIMING_CORE_HPP
