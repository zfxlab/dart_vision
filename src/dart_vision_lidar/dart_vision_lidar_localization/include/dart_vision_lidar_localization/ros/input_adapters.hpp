#ifndef DART_VISION_LIDAR_LOCALIZATION_ROS_INPUT_ADAPTERS_HPP
#define DART_VISION_LIDAR_LOCALIZATION_ROS_INPUT_ADAPTERS_HPP

#include <cstddef>
#include <livox_interfaces/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "dart_vision_lidar_localization/core/cloud_utils.hpp"

namespace dart_vision::lidar {

/**
 * @brief Result of converting a ROS point-cloud message to the internal cloud type.
 *
 * A successful conversion has an empty `error`. Non-fatal inconsistencies, such
 * as a Livox CustomMsg whose point_num differs from points.size(), are reported
 * in `warning`; conversion still uses the actual points array.
 */
struct CloudConversionResult {
    bool success{false};
    PointCloud cloud;
    std::string error;
    std::string warning;
    std::size_t declared_point_count{0U};
    std::size_t converted_point_count{0U};
    bool point_count_mismatch{false};

    [[nodiscard]] bool ok() const noexcept {
        return success;
    }
};

/**
 * @brief Convert sensor_msgs/PointCloud2 to pcl::PointCloud<pcl::PointXYZI>.
 *
 * Fields are located by name, rather than by assuming a padded PCL memory
 * layout. This supports the packed 18-byte Livox layout
 * (x/y/z/intensity/tag/line), ordinary PointXYZ and ordinary PointXYZI clouds.
 * x, y and z are required FLOAT32 or FLOAT64 scalar fields. intensity is
 * optional and defaults to zero; when present, any scalar PointField numeric
 * type is accepted. Organized clouds and row padding are read correctly.
 *
 * Both little- and big-endian payloads are supported. The ROS frame_id and
 * timestamp are copied to the PCL header (whose timestamp unit is microseconds).
 * Non-finite coordinates are deliberately retained for the preprocessing stage.
 */
[[nodiscard]] CloudConversionResult
convertPointCloud2(const sensor_msgs::msg::PointCloud2& message);

/**
 * @brief Convert a Livox CustomMsg to pcl::PointCloud<pcl::PointXYZI>.
 *
 * Livox coordinates are already expressed in metres. reflectivity is mapped to
 * intensity. points.size(), not point_num, controls conversion; a disagreement
 * is exposed in the returned diagnostics without rejecting the message.
 */
[[nodiscard]] CloudConversionResult
convertLivoxCustomMsg(const livox_interfaces::msg::CustomMsg& message);

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_LOCALIZATION_ROS_INPUT_ADAPTERS_HPP
