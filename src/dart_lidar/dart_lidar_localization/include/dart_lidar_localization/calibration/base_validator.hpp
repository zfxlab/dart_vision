#ifndef DART_LIDAR_LOCALIZATION_VALIDATION_REGISTRATION_VALIDATOR_HPP
#define DART_LIDAR_LOCALIZATION_VALIDATION_REGISTRATION_VALIDATOR_HPP

#include "dart_lidar_localization/localization_types.hpp"

namespace dart_vision::lidar::localization {

class BaseValidator {
public:
    [[nodiscard]] BaseRegistrationMetrics
    evaluate(const PointCloud::ConstPtr& source,
             const PointCloud::ConstPtr& target,
             const Eigen::Isometry3d& target_from_source,
             const Eigen::Isometry3d& initial_target_from_source,
             const BaseValidationParameters& parameters) const;

    [[nodiscard]] bool accept(const BaseRegistrationMetrics& metrics,
                              const BaseValidationParameters& parameters,
                              std::string& rejection_reason) const;
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_VALIDATION_REGISTRATION_VALIDATOR_HPP
