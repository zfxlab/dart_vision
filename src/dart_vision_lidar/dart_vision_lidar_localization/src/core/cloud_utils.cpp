#include "dart_vision_lidar_localization/core/cloud_utils.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace dart_vision::lidar {
namespace {

void validateTransform(const Eigen::Isometry3d& t_target_source) {
    if (!t_target_source.matrix().allFinite()) {
        throw std::invalid_argument("t_target_source must contain only finite values.");
    }
}

} // namespace

bool isPointFinite(const PointT& point) noexcept {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

PointT transformPoint(const PointT& point_source, const Eigen::Isometry3d& t_target_source) {
    validateTransform(t_target_source);

    PointT point_target = point_source;
    const Eigen::Vector3d position_target =
        t_target_source * Eigen::Vector3d{point_source.x, point_source.y, point_source.z};
    point_target.x = static_cast<float>(position_target.x());
    point_target.y = static_cast<float>(position_target.y());
    point_target.z = static_cast<float>(position_target.z());
    return point_target;
}

PointCloud transformPointCloud(const PointCloud& cloud_source,
                               const Eigen::Isometry3d& t_target_source,
                               const std::string& target_frame_id) {
    validateTransform(t_target_source);

    PointCloud cloud_target;
    cloud_target.header = cloud_source.header;
    cloud_target.header.frame_id = target_frame_id;
    cloud_target.points.reserve(cloud_source.size());

    for (const PointT& point_source : cloud_source.points) {
        PointT point_target = point_source;
        const Eigen::Vector3d position_target =
            t_target_source * Eigen::Vector3d{point_source.x, point_source.y, point_source.z};
        point_target.x = static_cast<float>(position_target.x());
        point_target.y = static_cast<float>(position_target.y());
        point_target.z = static_cast<float>(position_target.z());
        cloud_target.push_back(point_target);
    }

    const Eigen::Vector3d sensor_origin_source =
        cloud_source.sensor_origin_.head<3>().cast<double>();
    const Eigen::Vector3d sensor_origin_target = t_target_source * sensor_origin_source;
    cloud_target.sensor_origin_ = Eigen::Vector4f{static_cast<float>(sensor_origin_target.x()),
                                                  static_cast<float>(sensor_origin_target.y()),
                                                  static_cast<float>(sensor_origin_target.z()),
                                                  1.0F};

    const Eigen::Quaterniond q_target_source{t_target_source.linear()};
    const Eigen::Quaterniond q_source_sensor = cloud_source.sensor_orientation_.cast<double>();
    cloud_target.sensor_orientation_ = (q_target_source * q_source_sensor).cast<float>();

    finalizePointCloudMetadata(cloud_target);
    return cloud_target;
}

void finalizePointCloudMetadata(PointCloud& cloud) {
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1U;
    cloud.is_dense = true;

    for (const PointT& point : cloud.points) {
        if (!isPointFinite(point)) {
            cloud.is_dense = false;
            break;
        }
    }
}

} // namespace dart_vision::lidar
