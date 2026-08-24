#ifndef DART_VISION_LIDAR_LOCALIZATION_CORE_CLOUD_UTILS_HPP
#define DART_VISION_LIDAR_LOCALIZATION_CORE_CLOUD_UTILS_HPP

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace dart_vision::lidar {

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

/// Returns true when the XYZ coordinates of a point are finite.
[[nodiscard]] bool isPointFinite(const PointT& point) noexcept;

/**
 * @brief Transform one point from source coordinates into target coordinates.
 *
 * The direction is deliberately encoded in the parameter name:
 * `p_target = t_target_source * p_source`. Intensity is copied unchanged.
 *
 * @throws std::invalid_argument if t_target_source contains a non-finite value.
 */
[[nodiscard]] PointT transformPoint(const PointT& point_source,
                                    const Eigen::Isometry3d& t_target_source);

/**
 * @brief Transform a cloud from source coordinates into target coordinates.
 *
 * The direction is `p_target = t_target_source * p_source`. The output PCL
 * header is copied from cloud_source and its frame_id is replaced with
 * target_frame_id. Pass an empty target_frame_id only for frame-free test data.
 * The sensor origin and orientation metadata are transformed consistently.
 *
 * @throws std::invalid_argument if t_target_source contains a non-finite value.
 */
[[nodiscard]] PointCloud transformPointCloud(const PointCloud& cloud_source,
                                             const Eigen::Isometry3d& t_target_source,
                                             const std::string& target_frame_id);

/// Mark a cloud unorganized and update width, height, and is_dense from its points.
void finalizePointCloudMetadata(PointCloud& cloud);

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_LOCALIZATION_CORE_CLOUD_UTILS_HPP
