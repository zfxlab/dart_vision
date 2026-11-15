#pragma once

#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>

namespace dart_vision::camera {
struct PixelCenterEmaConfig {
    bool enabled{true};
    double tau_x_s{1.0};
    double tau_y_s{1.0};
    double reset_gap_s{2.0};

    [[nodiscard]] bool isValid() const noexcept;
};

// Double-precision state; the caller resets on invalid detections or geometry/target changes.
class PixelCenterEma {
  public:
    explicit PixelCenterEma(const PixelCenterEmaConfig& config = {});
    void configure(const PixelCenterEmaConfig& config);
    void reset() noexcept;
    [[nodiscard]] const PixelCenterEmaConfig& config() const noexcept;
    cv::Point2f update(const cv::Point2f& center, std::int64_t stamp_ns);

  private:
    PixelCenterEmaConfig config_;
    cv::Point2d center_{};
    std::optional<std::int64_t> stamp_ns_;
};
} // namespace dart_vision::camera
