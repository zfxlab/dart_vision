#ifndef DART_AIMING_AIMING_CORE_HPP
#define DART_AIMING_AIMING_CORE_HPP

#include <cstddef>
#include <deque>
#include <optional>

namespace dart_vision::aiming {

/// 输出参考坐标系中的瞄准结果，角度单位为弧度，距离单位为米。
struct Aim {
    double yaw_error_rad{};
    double distance_m{};
};

/// 输入坐标为 x 前、y 左、z 上；距离只计算 XY 水平分量，偏转角向左为正。
[[nodiscard]] std::optional<Aim> solve(double x, double y, double z) noexcept;

/// 对几何解应用离线拟合得到的残差补偿；当前为恒等映射，拟合确定后在实现中填写公式。
[[nodiscard]] std::optional<Aim> applyFittedCorrection(const Aim& input) noexcept;

/// 在拟合完成后加入控制器给出的飞镖固定偏角。
[[nodiscard]] std::optional<Aim> applyDartOffset(const Aim& input, double offset_rad) noexcept;

/// 对新的有效距离取滑动平均；窗口填满前不返回输出。
class DistanceMovingAverage {
  public:
    explicit DistanceMovingAverage(int window_frames);
    void reset() noexcept;
    [[nodiscard]] std::optional<double> update(double distance_m);

  private:
    std::size_t window_frames_;
    std::deque<double> samples_;
};

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
