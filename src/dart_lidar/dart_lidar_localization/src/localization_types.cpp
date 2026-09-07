#include "dart_lidar_localization/localization_types.hpp"

#include <stdexcept>

namespace dart_vision::lidar::localization {

const char* baseStatusMessage(const BaseRegistrationStatus status) noexcept {
    switch (status) {
        case BaseRegistrationStatus::kSuccess:
            return "success";
        case BaseRegistrationStatus::kInvalidInput:
            return "invalid input";
        case BaseRegistrationStatus::kCoarseFailed:
            return "coarse registration failed";
        case BaseRegistrationStatus::kFineFailed:
            return "fine registration failed";
        case BaseRegistrationStatus::kQualityRejected:
            return "registration rejected by quality validation";
        case BaseRegistrationStatus::kTimeBudgetExceeded:
            return "registration time budget exceeded";
        case BaseRegistrationStatus::kInternalError:
            return "internal registration error";
        case BaseRegistrationStatus::kTransformUnavailable:
            return "reference transform unavailable";
    }
    return "unknown registration status";
}

DofMode parseDofMode(const std::string& value) {
    if (value == "six_dof") {
        return DofMode::kSixDof;
    }
    if (value == "xyzyaw") {
        return DofMode::kXyzYaw;
    }
    if (value == "xyyaw") {
        return DofMode::kXyYaw;
    }
    throw std::invalid_argument("constraints.dof must be six_dof, xyzyaw or xyyaw");
}

} // namespace dart_vision::lidar::localization
