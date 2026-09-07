#include "dart_lidar_preprocessing/livox_preprocessor_node.hpp"

#include <cmath>
#include <functional>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <utility>

namespace dart_vision::lidar {
namespace {

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

void finalizeCloud(PointCloud& cloud) {
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1U;
    cloud.is_dense = true;
}

} // namespace

LivoxPreprocessorNode::LivoxPreprocessorNode(const rclcpp::NodeOptions& options)
    : Node("livox_preprocessor_node", options) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "lidar/preprocessed");
    min_distance_m_ = declare_parameter<double>("min_distance_m", 20.0);
    max_distance_m_ = declare_parameter<double>("max_distance_m", 30.0);
    voxel_grid_enabled_ = declare_parameter<bool>("voxel_grid_enabled", true);
    voxel_leaf_size_m_ = declare_parameter<double>("voxel_leaf_size_m", 0.01);

    if (input_topic_.empty() || output_topic_.empty() || !std::isfinite(min_distance_m_) ||
        min_distance_m_ < 0.0 || !std::isfinite(max_distance_m_) ||
        max_distance_m_ < min_distance_m_ || !std::isfinite(voxel_leaf_size_m_) ||
        voxel_leaf_size_m_ <= 0.0) {
        throw std::invalid_argument("Invalid Livox preprocessor parameter");
    }

    cloud_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    cloud_subscription_ = create_subscription<livox_interfaces::msg::CustomMsg>(
        input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&LivoxPreprocessorNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Livox preprocessing %s -> %s, range=[%.2f, %.2f]m, voxel=%s %.3fm",
                input_topic_.c_str(),
                output_topic_.c_str(),
                min_distance_m_,
                max_distance_m_,
                voxel_grid_enabled_ ? "on" : "off",
                voxel_leaf_size_m_);
}

void LivoxPreprocessorNode::cloudCallback(
    const livox_interfaces::msg::CustomMsg::ConstSharedPtr& message) {
    PointCloud::Ptr filtered(new PointCloud);
    filtered->reserve(message->points.size());

    const double minimum_squared = min_distance_m_ * min_distance_m_;
    const double maximum_squared = max_distance_m_ * max_distance_m_;

    for (const auto& source : message->points) {
        if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) {
            continue;
        }
        const double distance_squared =
            static_cast<double>(source.x * source.x + source.y * source.y + source.z * source.z);
        if (distance_squared < minimum_squared || distance_squared > maximum_squared) {
            continue;
        }

        PointT point;
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        point.intensity = static_cast<float>(source.reflectivity);
        filtered->push_back(point);
    }
    finalizeCloud(*filtered);

    if (voxel_grid_enabled_ && !filtered->empty()) {
        PointCloud::Ptr downsampled(new PointCloud);
        pcl::VoxelGrid<PointT> voxel;
        const float leaf = static_cast<float>(voxel_leaf_size_m_);

        voxel.setInputCloud(filtered);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*downsampled);

        finalizeCloud(*downsampled);
        filtered = std::move(downsampled);
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header = message->header;
    cloud_publisher_->publish(output);
}

} // namespace dart_vision::lidar
