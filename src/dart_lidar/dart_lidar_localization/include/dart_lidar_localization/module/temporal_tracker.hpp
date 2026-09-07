#ifndef DART_LIDAR_LOCALIZATION_MODULE_TEMPORAL_TRACKER_HPP
#define DART_LIDAR_LOCALIZATION_MODULE_TEMPORAL_TRACKER_HPP

#include "dart_lidar_localization/localization_types.hpp"

#include <optional>

namespace dart_vision::lidar::localization {

class TemporalTracker {
public:
    explicit TemporalTracker(double max_position_jump_m);

    [[nodiscard]] ModuleLocalizationResult filter(ModuleLocalizationResult result);
    void reset() noexcept;

private:
    double max_position_jump_m_{0.0};
    std::optional<double> last_position_m_;
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_MODULE_TEMPORAL_TRACKER_HPP
