#include "dart_vision_lidar_localization/ros/input_adapters.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_field.hpp>
#include <sstream>
#include <string_view>
#include <type_traits>

namespace dart_vision::lidar {
namespace {

using PointField = sensor_msgs::msg::PointField;

[[nodiscard]] bool
checkedMultiply(const std::size_t lhs, const std::size_t rhs, std::size_t* const result) noexcept {
    if (lhs != 0U && rhs > std::numeric_limits<std::size_t>::max() / lhs) {
        return false;
    }
    *result = lhs * rhs;
    return true;
}

[[nodiscard]] std::size_t datatypeSize(const std::uint8_t datatype) noexcept {
    switch (datatype) {
        case PointField::INT8:
        case PointField::UINT8:
            return 1U;
        case PointField::INT16:
        case PointField::UINT16:
            return 2U;
        case PointField::INT32:
        case PointField::UINT32:
        case PointField::FLOAT32:
            return 4U;
        case PointField::FLOAT64:
            return 8U;
        default:
            return 0U;
    }
}

[[nodiscard]] bool hostIsBigEndian() noexcept {
    const std::uint16_t value = 0x0102U;
    std::array<std::uint8_t, sizeof(value)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(value));
    return bytes.front() == 0x01U;
}

template <typename Scalar>
[[nodiscard]] Scalar readScalar(const std::uint8_t* const source,
                                const bool source_is_big_endian) noexcept {
    static_assert(std::is_arithmetic_v<Scalar>);
    std::array<std::uint8_t, sizeof(Scalar)> bytes{};
    std::copy_n(source, sizeof(Scalar), bytes.begin());
    if (source_is_big_endian != hostIsBigEndian() && sizeof(Scalar) > 1U) {
        std::reverse(bytes.begin(), bytes.end());
    }

    Scalar value{};
    std::memcpy(&value, bytes.data(), sizeof(value));
    return value;
}

[[nodiscard]] double readNumericField(const std::uint8_t* const source,
                                      const std::uint8_t datatype,
                                      const bool source_is_big_endian) noexcept {
    switch (datatype) {
        case PointField::INT8:
            return static_cast<double>(readScalar<std::int8_t>(source, source_is_big_endian));
        case PointField::UINT8:
            return static_cast<double>(readScalar<std::uint8_t>(source, source_is_big_endian));
        case PointField::INT16:
            return static_cast<double>(readScalar<std::int16_t>(source, source_is_big_endian));
        case PointField::UINT16:
            return static_cast<double>(readScalar<std::uint16_t>(source, source_is_big_endian));
        case PointField::INT32:
            return static_cast<double>(readScalar<std::int32_t>(source, source_is_big_endian));
        case PointField::UINT32:
            return static_cast<double>(readScalar<std::uint32_t>(source, source_is_big_endian));
        case PointField::FLOAT32:
            return static_cast<double>(readScalar<float>(source, source_is_big_endian));
        case PointField::FLOAT64:
            return readScalar<double>(source, source_is_big_endian);
        default:
            return 0.0;
    }
}

[[nodiscard]] bool copyHeader(const std_msgs::msg::Header& source,
                              pcl::PCLHeader* const target,
                              std::string* const error) {
    if (source.stamp.sec < 0) {
        *error = "negative ROS timestamp cannot be represented by pcl::PCLHeader";
        return false;
    }
    if (source.stamp.nanosec >= 1000000000U) {
        *error = "ROS timestamp nanosec must be less than 1000000000";
        return false;
    }

    target->seq = 0U;
    target->frame_id = source.frame_id;
    target->stamp = static_cast<std::uint64_t>(source.stamp.sec) * 1000000ULL +
                    static_cast<std::uint64_t>(source.stamp.nanosec) / 1000ULL;
    return true;
}

[[nodiscard]] const PointField* findUniqueField(const sensor_msgs::msg::PointCloud2& message,
                                                const std::string_view name,
                                                std::string* const error) {
    const PointField* result = nullptr;
    for (const PointField& field : message.fields) {
        if (field.name != name) {
            continue;
        }
        if (result != nullptr) {
            *error = "PointCloud2 contains duplicate field '" + std::string{name} + "'";
            return nullptr;
        }
        result = &field;
    }
    return result;
}

[[nodiscard]] bool validateAllFields(const sensor_msgs::msg::PointCloud2& message,
                                     std::string* const error) {
    for (const PointField& field : message.fields) {
        const std::size_t scalar_size = datatypeSize(field.datatype);
        if (scalar_size == 0U) {
            *error = "PointCloud2 field '" + field.name + "' has unsupported datatype " +
                     std::to_string(field.datatype);
            return false;
        }
        if (field.count == 0U) {
            *error = "PointCloud2 field '" + field.name + "' has zero count";
            return false;
        }

        std::size_t field_bytes = 0U;
        if (!checkedMultiply(static_cast<std::size_t>(field.count), scalar_size, &field_bytes) ||
            static_cast<std::size_t>(field.offset) >
                std::numeric_limits<std::size_t>::max() - field_bytes ||
            static_cast<std::size_t>(field.offset) + field_bytes > message.point_step) {
            *error = "PointCloud2 field '" + field.name + "' extends beyond point_step";
            return false;
        }
    }
    return true;
}

[[nodiscard]] bool validateCoordinateField(const PointField* const field,
                                           const std::string_view name,
                                           std::string* const error) {
    if (field == nullptr) {
        if (error->empty()) {
            *error = "PointCloud2 is missing required field '" + std::string{name} + "'";
        }
        return false;
    }
    if (field->count != 1U) {
        *error = "PointCloud2 coordinate field '" + std::string{name} + "' must have count 1";
        return false;
    }
    if (field->datatype != PointField::FLOAT32 && field->datatype != PointField::FLOAT64) {
        *error =
            "PointCloud2 coordinate field '" + std::string{name} + "' must be FLOAT32 or FLOAT64";
        return false;
    }
    return true;
}

[[nodiscard]] bool validateIntensityField(const PointField* const field, std::string* const error) {
    if (field == nullptr) {
        return error->empty();
    }
    if (field->count != 1U) {
        *error = "PointCloud2 intensity field must have count 1";
        return false;
    }
    if (datatypeSize(field->datatype) == 0U) {
        *error = "PointCloud2 intensity field has an unsupported datatype";
        return false;
    }
    return true;
}

} // namespace

CloudConversionResult convertPointCloud2(const sensor_msgs::msg::PointCloud2& message) {
    CloudConversionResult result;
    if (!copyHeader(message.header, &result.cloud.header, &result.error)) {
        return result;
    }

    std::size_t point_count = 0U;
    if (!checkedMultiply(static_cast<std::size_t>(message.width),
                         static_cast<std::size_t>(message.height),
                         &point_count)) {
        result.error = "PointCloud2 width * height overflows size_t";
        return result;
    }
    result.declared_point_count = point_count;

    std::size_t minimum_row_step = 0U;
    if (!checkedMultiply(static_cast<std::size_t>(message.width),
                         static_cast<std::size_t>(message.point_step),
                         &minimum_row_step)) {
        result.error = "PointCloud2 width * point_step overflows size_t";
        return result;
    }
    if (static_cast<std::size_t>(message.row_step) < minimum_row_step) {
        result.error = "PointCloud2 row_step is smaller than width * point_step";
        return result;
    }

    std::size_t expected_data_size = 0U;
    if (!checkedMultiply(static_cast<std::size_t>(message.row_step),
                         static_cast<std::size_t>(message.height),
                         &expected_data_size)) {
        result.error = "PointCloud2 row_step * height overflows size_t";
        return result;
    }
    if (message.data.size() != expected_data_size) {
        std::ostringstream stream;
        stream << "PointCloud2 data size is " << message.data.size() << ", expected "
               << expected_data_size << " from row_step * height";
        result.error = stream.str();
        return result;
    }

    if (!validateAllFields(message, &result.error)) {
        return result;
    }

    const PointField* const x_field = findUniqueField(message, "x", &result.error);
    if (!validateCoordinateField(x_field, "x", &result.error)) {
        return result;
    }
    const PointField* const y_field = findUniqueField(message, "y", &result.error);
    if (!validateCoordinateField(y_field, "y", &result.error)) {
        return result;
    }
    const PointField* const z_field = findUniqueField(message, "z", &result.error);
    if (!validateCoordinateField(z_field, "z", &result.error)) {
        return result;
    }
    const PointField* const intensity_field = findUniqueField(message, "intensity", &result.error);
    if (!validateIntensityField(intensity_field, &result.error)) {
        return result;
    }

    result.cloud.points.reserve(point_count);
    for (std::size_t row = 0U; row < message.height; ++row) {
        const std::size_t row_offset = row * static_cast<std::size_t>(message.row_step);
        for (std::size_t column = 0U; column < message.width; ++column) {
            const std::uint8_t* const point_data =
                message.data.data() + row_offset +
                column * static_cast<std::size_t>(message.point_step);

            PointT point;
            point.x = static_cast<float>(readNumericField(
                point_data + x_field->offset, x_field->datatype, message.is_bigendian));
            point.y = static_cast<float>(readNumericField(
                point_data + y_field->offset, y_field->datatype, message.is_bigendian));
            point.z = static_cast<float>(readNumericField(
                point_data + z_field->offset, z_field->datatype, message.is_bigendian));
            point.intensity =
                intensity_field == nullptr
                    ? 0.0F
                    : static_cast<float>(readNumericField(point_data + intensity_field->offset,
                                                          intensity_field->datatype,
                                                          message.is_bigendian));
            result.cloud.push_back(point);
        }
    }

    finalizePointCloudMetadata(result.cloud);
    result.converted_point_count = result.cloud.size();
    result.success = true;
    return result;
}

CloudConversionResult convertLivoxCustomMsg(const livox_interfaces::msg::CustomMsg& message) {
    CloudConversionResult result;
    if (!copyHeader(message.header, &result.cloud.header, &result.error)) {
        return result;
    }

    result.declared_point_count = static_cast<std::size_t>(message.point_num);
    result.point_count_mismatch = result.declared_point_count != message.points.size();
    if (result.point_count_mismatch) {
        std::ostringstream stream;
        stream << "Livox CustomMsg point_num is " << result.declared_point_count
               << " but points.size() is " << message.points.size()
               << "; converted the points array";
        result.warning = stream.str();
    }

    result.cloud.points.reserve(message.points.size());
    for (const auto& source : message.points) {
        PointT point;
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        point.intensity = static_cast<float>(source.reflectivity);
        result.cloud.push_back(point);
    }

    finalizePointCloudMetadata(result.cloud);
    result.converted_point_count = result.cloud.size();
    result.success = true;
    return result;
}

} // namespace dart_vision::lidar
