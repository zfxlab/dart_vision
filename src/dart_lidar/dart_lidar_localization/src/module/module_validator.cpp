#include "dart_lidar_localization/module/module_validator.hpp"

#include <cmath>
#include <sstream>

namespace dart_vision::lidar::localization {

bool ModuleValidator::accept(const ModuleLocalizationResult& result,
                             const ModuleLocalizationParameters& parameters,
                             std::string& reason) const {
    std::ostringstream output;
    if (!std::isfinite(result.position_m) || !std::isfinite(result.metrics.rmse_m)) {
        output << "module solution is not finite";
    } else if (result.position_m < parameters.min_position_m ||
               result.position_m > parameters.max_position_m) {
        output << "module position is outside the rail limits";
    } else if (result.metrics.rmse_m > parameters.max_rmse_m) {
        output << "module rmse " << result.metrics.rmse_m << "m exceeds " << parameters.max_rmse_m
               << "m";
    } else if (result.metrics.overlap_ratio < parameters.min_overlap_ratio) {
        output << "module overlap " << result.metrics.overlap_ratio << " is below "
               << parameters.min_overlap_ratio;
    } else if (result.metrics.correspondence_count < parameters.min_correspondences) {
        output << "module correspondences " << result.metrics.correspondence_count << " is below "
               << parameters.min_correspondences;
    } else {
        reason.clear();
        return true;
    }
    reason = output.str();
    return false;
}

} // namespace dart_vision::lidar::localization
