#ifndef DART_VISION_LIDAR_LOCALIZATION_CORE_TEMPORAL_GATE_HPP
#define DART_VISION_LIDAR_LOCALIZATION_CORE_TEMPORAL_GATE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>

#include "dart_vision_lidar_localization/core/alignment_metrics.hpp"

namespace dart_vision::lidar {

struct TemporalGateConfig {
    std::size_t required_consecutive_frames{3};
    double max_translation_jump_m{0.08};
    double max_rotation_jump_rad{0.15};
};

struct TemporalGateInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    int state{-1};
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    double confidence{0.0};
    LocalizationStatus status{LocalizationStatus::kTemporalUnconfirmed};
};

struct TemporalGateOutput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    int state{-1};
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    double confidence{0.0};
    std::size_t consecutive_frames{0};
    LocalizationStatus status{LocalizationStatus::kTemporalUnconfirmed};
};

// A bad or missing measurement is invalid immediately and resets confirmation.
// This intentionally never republishes a stale pose as valid.
class TemporalGate {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit TemporalGate(TemporalGateConfig config = {});

    [[nodiscard]] TemporalGateOutput update(const TemporalGateInput& input);
    void reset() noexcept;
    void setConfig(const TemporalGateConfig& config);
    [[nodiscard]] const TemporalGateConfig& config() const noexcept;

private:
    [[nodiscard]] bool configIsValid() const noexcept;
    [[nodiscard]] bool isContinuous(const Eigen::Isometry3d& pose) const noexcept;

    TemporalGateConfig config_{};
    bool has_pending_candidate_{false};
    int pending_state_{-1};
    Eigen::Isometry3d pending_pose_{Eigen::Isometry3d::Identity()};
    std::size_t consecutive_frames_{0};
};

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_LOCALIZATION_CORE_TEMPORAL_GATE_HPP
