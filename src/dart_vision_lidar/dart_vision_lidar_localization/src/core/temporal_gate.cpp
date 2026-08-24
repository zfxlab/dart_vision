#include "dart_vision_lidar_localization/core/temporal_gate.hpp"

#include <algorithm>
#include <cmath>

namespace dart_vision::lidar {

TemporalGate::TemporalGate(TemporalGateConfig config) : config_(config) {}

TemporalGateOutput TemporalGate::update(const TemporalGateInput& input) {
    TemporalGateOutput output;
    output.state = input.state;
    output.pose = input.pose;
    output.confidence = input.confidence;

    if (!configIsValid()) {
        reset();
        output.status = LocalizationStatus::kInvalidConfiguration;
        return output;
    }
    if (!input.valid || input.state < 0) {
        reset();
        output.status = input.status;
        return output;
    }
    if (!isFiniteTransform(input.pose)) {
        reset();
        output.status = LocalizationStatus::kNonFiniteTransform;
        return output;
    }

    bool discontinuity = false;
    if (!has_pending_candidate_ || pending_state_ != input.state) {
        has_pending_candidate_ = true;
        pending_state_ = input.state;
        pending_pose_ = input.pose;
        consecutive_frames_ = 1U;
    } else if (!isContinuous(input.pose)) {
        pending_pose_ = input.pose;
        consecutive_frames_ = 1U;
        discontinuity = true;
    } else {
        pending_pose_ = input.pose;
        ++consecutive_frames_;
    }

    output.consecutive_frames = consecutive_frames_;
    if (discontinuity) {
        output.status = LocalizationStatus::kTemporalDiscontinuity;
        return output;
    }
    if (consecutive_frames_ < std::max<std::size_t>(config_.required_consecutive_frames, 1U)) {
        output.status = LocalizationStatus::kTemporalUnconfirmed;
        return output;
    }

    output.valid = true;
    output.status = LocalizationStatus::kValid;
    return output;
}

void TemporalGate::reset() noexcept {
    has_pending_candidate_ = false;
    pending_state_ = -1;
    pending_pose_.setIdentity();
    consecutive_frames_ = 0U;
}

void TemporalGate::setConfig(const TemporalGateConfig& config) {
    config_ = config;
    reset();
}

const TemporalGateConfig& TemporalGate::config() const noexcept {
    return config_;
}

bool TemporalGate::configIsValid() const noexcept {
    return config_.required_consecutive_frames > 0U &&
           !std::isnan(config_.max_translation_jump_m) && config_.max_translation_jump_m >= 0.0 &&
           !std::isnan(config_.max_rotation_jump_rad) && config_.max_rotation_jump_rad >= 0.0;
}

bool TemporalGate::isContinuous(const Eigen::Isometry3d& pose) const noexcept {
    const double translation_jump = (pose.translation() - pending_pose_.translation()).norm();
    const double rotation_jump = rotationAngle(pending_pose_.linear().transpose() * pose.linear());
    return translation_jump <= config_.max_translation_jump_m &&
           rotation_jump <= config_.max_rotation_jump_rad;
}

} // namespace dart_vision::lidar
