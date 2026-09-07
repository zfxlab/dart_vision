#ifndef DART_LIDAR_LOCALIZATION_PIPELINE_LOCALIZATION_PIPELINE_HPP
#define DART_LIDAR_LOCALIZATION_PIPELINE_LOCALIZATION_PIPELINE_HPP

#include "dart_lidar_localization/base/base_registrar.hpp"
#include "dart_lidar_localization/module/rail_module_localizer.hpp"
#include "dart_lidar_localization/module/temporal_tracker.hpp"

#include <memory>

namespace dart_vision::lidar::localization {

class LocalizationPipeline {
public:
    LocalizationPipeline(BaseRegistrationParameters base_parameters,
                         ModuleLocalizationParameters module_parameters,
                         PointCloud::ConstPtr base_model,
                         PointCloud::ConstPtr module_model);

    [[nodiscard]] LocalizationResult localize(
        const PointCloud::ConstPtr& observation,
        const Eigen::Isometry3d& initial_base_from_observation,
        const Eigen::Isometry3d& reference_from_current_base,
        bool observation_frame_is_reference,
        const Eigen::Isometry3d& base_from_rail);

    void resetModuleHistory() noexcept;

private:
    BaseRegistrar base_registrar_;
    std::unique_ptr<RailModuleLocalizer> module_localizer_;
    TemporalTracker module_tracker_;
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_PIPELINE_LOCALIZATION_PIPELINE_HPP
