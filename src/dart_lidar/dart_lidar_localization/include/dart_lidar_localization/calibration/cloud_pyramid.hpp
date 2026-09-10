#ifndef DART_LIDAR_LOCALIZATION_CLOUD_PYRAMID_HPP
#define DART_LIDAR_LOCALIZATION_CLOUD_PYRAMID_HPP

#include <vector>

#include "dart_lidar_localization/localization_types.hpp"

namespace dart_vision::lidar::localization {

class CloudPyramid {
public:
    [[nodiscard]] static PointCloud::Ptr downsample(const PointCloud::ConstPtr& cloud,
                                                    double leaf_size_m);
    [[nodiscard]] static std::vector<PointCloud::Ptr>
    build(const PointCloud::ConstPtr& cloud, const std::vector<double>& leaf_sizes_m);
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_CLOUD_PYRAMID_HPP
