#ifndef DART_LIDAR_LOCALIZATION_MODULE_RAIL_MODULE_LOCALIZER_HPP
#define DART_LIDAR_LOCALIZATION_MODULE_RAIL_MODULE_LOCALIZER_HPP

#include "dart_lidar_localization/localization_types.hpp"
#include "dart_lidar_localization/module/module_validator.hpp"

namespace dart_vision::lidar::localization {

class RailModuleLocalizer {
public:
    RailModuleLocalizer(ModuleLocalizationParameters parameters, PointCloud::ConstPtr module_model);

    [[nodiscard]] ModuleLocalizationResult
    locate(const PointCloud::ConstPtr& observation_in_reference,
           const Eigen::Isometry3d& reference_from_rail) const;

private:
    ModuleLocalizationParameters parameters_;
    PointCloud::ConstPtr module_model_;
    Eigen::Vector3d model_min_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d model_max_{Eigen::Vector3d::Zero()};
    ModuleValidator validator_;
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_MODULE_RAIL_MODULE_LOCALIZER_HPP
