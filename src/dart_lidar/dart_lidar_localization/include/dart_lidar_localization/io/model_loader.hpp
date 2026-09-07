#ifndef DART_LIDAR_LOCALIZATION_IO_MODEL_LOADER_HPP
#define DART_LIDAR_LOCALIZATION_IO_MODEL_LOADER_HPP

#include "dart_lidar_localization/localization_types.hpp"

#include <string>

namespace dart_vision::lidar::localization {

class ModelLoader {
public:
    [[nodiscard]] static std::string resolvePath(const std::string& resource_path);
    [[nodiscard]] static PointCloud::Ptr load(const std::string& resource_path);
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_IO_MODEL_LOADER_HPP
