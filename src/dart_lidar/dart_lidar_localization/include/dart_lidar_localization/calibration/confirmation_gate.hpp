#pragma once
#include <cmath>
#include <cstdint>
#include <optional>

#include "dart_lidar_localization/localization_types.hpp"

namespace dart_vision::lidar::localization {
// 三次有效结果；至少间隔0.6秒，避免把高度重叠的窗口重复计为独立确认。
// 与首个结果比较，防止逐次微小漂移最终累积成大偏差。
class ConfirmationGate {
public:
    bool observe(const BaseRegistrationResult& result, std::int64_t stamp_ns) {
        if (stamp_ns <= 0 || !result.success()) {
            reset();
            return false;
        }
        if (last_seen_ > 0 && stamp_ns < last_seen_) {
            reset();
        } else if (stamp_ns == last_seen_) {
            return count_ >= 3;
        }
        last_seen_ = stamp_ns;
        const auto& pose = result.observation_from_model;
        if (anchor_) {
            const double angle =
                Eigen::AngleAxisd(anchor_->linear().transpose() * pose.linear()).angle();
            if ((anchor_->translation() - pose.translation()).norm() > 0.01 ||
                std::abs(angle) > 0.005) {
                reset();
                last_seen_ = stamp_ns;
            }
        }
        if (!anchor_) {
            anchor_ = pose;
            count_ = 1;
            last_counted_ = stamp_ns;
        } else if (stamp_ns - last_counted_ >= 600000000LL) {
            ++count_;
            last_counted_ = stamp_ns;
        }
        return count_ >= 3;
    }
    void reset() {
        anchor_.reset();
        last_seen_ = last_counted_ = 0;
        count_ = 0;
    }

private:
    std::optional<Eigen::Isometry3d> anchor_;
    std::int64_t last_seen_{0}, last_counted_{0};
    unsigned count_{0};
};
} // namespace dart_vision::lidar::localization
