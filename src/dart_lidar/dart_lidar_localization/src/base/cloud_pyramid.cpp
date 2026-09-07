#include "dart_lidar_localization/base/cloud_pyramid.hpp"

#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <stdexcept>

namespace dart_vision::lidar::localization {

PointCloud::Ptr CloudPyramid::downsample(const PointCloud::ConstPtr& cloud,
                                         const double leaf_size_m) {
    if (!cloud) {
        throw std::invalid_argument("Cannot downsample a null cloud");
    }
    if (!std::isfinite(leaf_size_m) || leaf_size_m <= 0.0) {
        throw std::invalid_argument("Voxel leaf size must be positive and finite");
    }

    PointCloud::Ptr result(new PointCloud);
    pcl::VoxelGrid<PointT> voxel;
    const float leaf = static_cast<float>(leaf_size_m);
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*result);
    return result;
}

std::vector<PointCloud::Ptr> CloudPyramid::build(const PointCloud::ConstPtr& cloud,
                                                 const std::vector<double>& leaf_sizes_m) {
    std::vector<PointCloud::Ptr> result;
    result.reserve(leaf_sizes_m.size());
    for (const double leaf_size : leaf_sizes_m) {
        result.push_back(downsample(cloud, leaf_size));
    }
    return result;
}

} // namespace dart_vision::lidar::localization
