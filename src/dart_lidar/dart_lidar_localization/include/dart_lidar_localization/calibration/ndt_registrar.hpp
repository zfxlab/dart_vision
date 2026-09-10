#ifndef DART_LIDAR_LOCALIZATION_COARSE_NDT_REGISTRAR_HPP
#define DART_LIDAR_LOCALIZATION_COARSE_NDT_REGISTRAR_HPP

#include "dart_lidar_localization/localization_types.hpp"

namespace dart_vision::lidar::localization {

class NdtRegistrar {
public:
    [[nodiscard]] BaseStageResult align(const PointCloud::ConstPtr& source,
                                        const PointCloud::ConstPtr& target,
                                        const Eigen::Isometry3d& initial_guess,
                                        const NdtLevelParameters& parameters) const;
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_COARSE_NDT_REGISTRAR_HPP
