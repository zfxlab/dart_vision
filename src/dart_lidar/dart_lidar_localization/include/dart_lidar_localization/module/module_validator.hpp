#ifndef DART_LIDAR_LOCALIZATION_MODULE_MODULE_VALIDATOR_HPP
#define DART_LIDAR_LOCALIZATION_MODULE_MODULE_VALIDATOR_HPP

#include "dart_lidar_localization/localization_types.hpp"

namespace dart_vision::lidar::localization {

class ModuleValidator {
public:
    [[nodiscard]] bool accept(const ModuleLocalizationResult& result,
                              const ModuleLocalizationParameters& parameters,
                              std::string& reason) const;
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_MODULE_MODULE_VALIDATOR_HPP
