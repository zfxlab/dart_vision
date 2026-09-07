#include "dart_lidar_localization/module/temporal_tracker.hpp"

#include <cmath>
#include <stdexcept>

namespace dart_vision::lidar::localization {

TemporalTracker::TemporalTracker(const double max_position_jump_m)
    : max_position_jump_m_(max_position_jump_m) {
    if (!std::isfinite(max_position_jump_m_)) {
        throw std::invalid_argument("module max_position_jump_m must be finite");
    }
}

ModuleLocalizationResult TemporalTracker::filter(ModuleLocalizationResult result) {
    if (!result.available) {
        return result;
    }
    if (max_position_jump_m_ > 0.0 && last_position_m_.has_value() &&
        std::abs(result.position_m - *last_position_m_) > max_position_jump_m_) {
        result.available = false;
        result.message = "module position jump exceeds the configured limit";
        return result;
    }
    last_position_m_ = result.position_m;
    return result;
}

void TemporalTracker::reset() noexcept {
    last_position_m_.reset();
}

} // namespace dart_vision::lidar::localization
