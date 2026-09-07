#ifndef DART_LIDAR_LOCALIZATION_REGISTRATION_PIPELINE_HPP
#define DART_LIDAR_LOCALIZATION_REGISTRATION_PIPELINE_HPP

#include "dart_lidar_localization/base/ndt_registrar.hpp"
#include "dart_lidar_localization/base/gicp_registrar.hpp"
#include "dart_lidar_localization/localization_types.hpp"
#include "dart_lidar_localization/base/base_validator.hpp"

#include <chrono>
#include <vector>

namespace dart_vision::lidar::localization {

class BaseRegistrar {
public:
    BaseRegistrar(BaseRegistrationParameters parameters, PointCloud::ConstPtr model);

    [[nodiscard]] BaseRegistrationResult align(
        const PointCloud::ConstPtr& observation,
        const Eigen::Isometry3d& initial_target_from_observation) const;

private:
    [[nodiscard]] Eigen::Isometry3d projectToAllowedDof(
        const Eigen::Isometry3d& target_from_observation,
        const Eigen::Isometry3d& initial_target_from_observation) const;
    [[nodiscard]] bool timeBudgetExceeded(
        const std::chrono::steady_clock::time_point& started_at) const;

    BaseRegistrationParameters parameters_;
    PointCloud::ConstPtr model_;
    std::vector<PointCloud::Ptr> coarse_models_;
    std::vector<PointCloud::Ptr> fine_models_;
    NdtRegistrar ndt_;
    GicpRegistrar gicp_;
    BaseValidator validator_;
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_REGISTRATION_PIPELINE_HPP
