#include "dart_lidar_localization/pipeline/localization_pipeline.hpp"

#include <pcl/common/transforms.h>
#include <stdexcept>
#include <utility>

namespace dart_vision::lidar::localization {

LocalizationPipeline::LocalizationPipeline(
    BaseRegistrationParameters base_parameters,
    ModuleLocalizationParameters module_parameters,
    PointCloud::ConstPtr base_model,
    PointCloud::ConstPtr module_model)
    : base_registrar_(std::move(base_parameters), std::move(base_model)),
      module_tracker_(module_parameters.max_position_jump_m) {
    if (module_parameters.enabled) {
        module_localizer_ = std::make_unique<RailModuleLocalizer>(
            std::move(module_parameters), std::move(module_model));
    }
}

LocalizationResult LocalizationPipeline::localize(
    const PointCloud::ConstPtr& observation,
    const Eigen::Isometry3d& initial_base_from_observation,
    const Eigen::Isometry3d& reference_from_current_base,
    const bool observation_frame_is_reference,
    const Eigen::Isometry3d& base_from_rail) {
    LocalizationResult result;
    result.base = base_registrar_.align(observation, initial_base_from_observation);
    if (!result.base.success()) {
        result.module.message = "base registration is unavailable";
        return result;
    }

    // observation_from_model是输入点云坐标系 <- 实际基地坐标系。
    result.reference_from_base = observation_frame_is_reference
                                     ? result.base.observation_from_model
                                     : reference_from_current_base *
                                           result.base.observation_from_model;

    if (!module_localizer_) {
        result.module.message = "moving-module localization is disabled";
        return result;
    }

    PointCloud::Ptr observation_in_base(new PointCloud);
    pcl::transformPointCloud(*observation,
                             *observation_in_base,
                             result.base.target_from_source.matrix().cast<float>());
    result.module = module_tracker_.filter(
        module_localizer_->locate(observation_in_base, base_from_rail));
    return result;
}

void LocalizationPipeline::resetModuleHistory() noexcept {
    module_tracker_.reset();
}

} // namespace dart_vision::lidar::localization
