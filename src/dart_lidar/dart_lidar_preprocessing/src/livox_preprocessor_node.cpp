#include "dart_lidar_preprocessing/livox_preprocessor_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utility>
#include <vector>

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
    : Node("livox_preprocessor_node", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "lidar/preprocessed");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_nominal_link");
    min_distance_m_ = declare_parameter<double>("min_distance_m", 20.0);
    max_distance_m_ = declare_parameter<double>("max_distance_m", 30.0);
    voxel_grid_enabled_ = declare_parameter<bool>("voxel_grid_enabled", true);
    voxel_leaf_size_m_ = declare_parameter<double>("voxel_leaf_size_m", 0.01);
    yaw_query_offset_s_ = declare_parameter<double>("motion_compensation.yaw_query_offset_s", 0.0);
    time_bin_s_ = declare_parameter<double>("motion_compensation.time_bin_s", 0.002);
    tf_wait_timeout_s_ = declare_parameter<double>("motion_compensation.tf_wait_timeout_s", 0.15);
    const auto max_pending =
        declare_parameter<std::int64_t>("motion_compensation.max_pending_clouds", 8);

    if (input_topic_.empty() || output_topic_.empty() || target_frame_.empty() ||
        !std::isfinite(min_distance_m_) || min_distance_m_ < 0.0 ||
        !std::isfinite(max_distance_m_) || max_distance_m_ < min_distance_m_ ||
        !std::isfinite(voxel_leaf_size_m_) || voxel_leaf_size_m_ <= 0.0 ||
        !std::isfinite(yaw_query_offset_s_) || !std::isfinite(time_bin_s_) || time_bin_s_ <= 0.0 ||
        !std::isfinite(tf_wait_timeout_s_) || tf_wait_timeout_s_ <= 0.0 || max_pending <= 0) {
        throw std::invalid_argument("Invalid Livox preprocessor parameter");
    }
    max_pending_clouds_ = static_cast<std::size_t>(max_pending);

    cloud_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    cloud_subscription_ = create_subscription<livox_interfaces::msg::CustomMsg>(
        input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&LivoxPreprocessorNode::cloudCallback, this, std::placeholders::_1));
    pending_timer_ =
        create_wall_timer(std::chrono::milliseconds(5),
                          std::bind(&LivoxPreprocessorNode::processPendingClouds, this));

    RCLCPP_INFO(get_logger(),
                "Livox preprocessing %s -> %s in %s, range=[%.2f, %.2f]m, "
                "yaw_query_offset=%.3fms bin=%.3fms voxel=%s %.3fm",
                input_topic_.c_str(),
                output_topic_.c_str(),
                target_frame_.c_str(),
                min_distance_m_,
                max_distance_m_,
                yaw_query_offset_s_ * 1000.0,
                time_bin_s_ * 1000.0,
                voxel_grid_enabled_ ? "on" : "off",
                voxel_leaf_size_m_);
}

void LivoxPreprocessorNode::cloudCallback(
    const livox_interfaces::msg::CustomMsg::ConstSharedPtr& message) {
    if (message->header.frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "Dropping Livox cloud with empty frame_id");
        return;
    }
    pending_clouds_.push_back(PendingCloud{message, now(), std::chrono::steady_clock::now()});
    while (pending_clouds_.size() > max_pending_clouds_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Dropping oldest Livox cloud because motion-compensation queue is full");
        pending_clouds_.pop_front();
    }
    processPendingClouds();
}

void LivoxPreprocessorNode::processPendingClouds() {
    while (!pending_clouds_.empty()) {
        if (preprocessAndPublish(pending_clouds_.front())) {
            pending_clouds_.pop_front();
            continue;
        }
        const double waited_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          pending_clouds_.front().received_steady_time)
                .count();
        if (waited_s < tf_wait_timeout_s_) {
            break;
        }
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Dropping Livox cloud after %.1fms: yaw/TF history does not cover all point times",
            waited_s * 1000.0);
        pending_clouds_.pop_front();
    }
}

bool LivoxPreprocessorNode::preprocessAndPublish(const PendingCloud& pending) {
    const auto& message = *pending.message;
    std::uint32_t maximum_offset_ns = 0U;
    for (const auto& point : message.points) {
        maximum_offset_ns = std::max(maximum_offset_ns, point.offset_time);
    }

    const std::int64_t bin_ns =
        std::max<std::int64_t>(1LL, static_cast<std::int64_t>(std::llround(time_bin_s_ * 1.0e9)));
    const std::int64_t query_offset_ns =
        static_cast<std::int64_t>(std::llround(yaw_query_offset_s_ * 1.0e9));
    const std::size_t bin_count = static_cast<std::size_t>(maximum_offset_ns / bin_ns) + 1U;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> transforms(
        bin_count, Eigen::Isometry3d::Identity());

    if (message.header.frame_id != target_frame_) {
        try {
            for (std::size_t index = 0U; index < bin_count; ++index) {
                const std::int64_t point_offset_ns =
                    std::min<std::int64_t>(static_cast<std::int64_t>(maximum_offset_ns),
                                           static_cast<std::int64_t>(index) * bin_ns + bin_ns / 2);
                const std::int64_t query_ns = pending.received_ros_time.nanoseconds() -
                                              static_cast<std::int64_t>(maximum_offset_ns) +
                                              point_offset_ns + query_offset_ns;
                if (query_ns <= 0) {
                    return false;
                }
                const rclcpp::Time query_time(query_ns, pending.received_ros_time.get_clock_type());
                const auto transform =
                    tf_buffer_.lookupTransform(target_frame_,
                                               message.header.frame_id,
                                               query_time,
                                               rclcpp::Duration::from_seconds(0.0));
                transforms[index] = tf2::transformToEigen(transform.transform);
            }
        } catch (const tf2::TransformException&) {
            return false;
        }
    }

    PointCloud::Ptr filtered(new PointCloud);
    filtered->reserve(message.points.size());

    const double minimum_squared = min_distance_m_ * min_distance_m_;
    const double maximum_squared = max_distance_m_ * max_distance_m_;

    for (const auto& source : message.points) {
        if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) {
            continue;
        }
        const double distance_squared =
            static_cast<double>(source.x * source.x + source.y * source.y + source.z * source.z);
        if (distance_squared < minimum_squared || distance_squared > maximum_squared) {
            continue;
        }

        const std::size_t bin_index = std::min<std::size_t>(
            transforms.size() - 1U, static_cast<std::size_t>(source.offset_time / bin_ns));
        const Eigen::Vector3d transformed =
            transforms[bin_index] * Eigen::Vector3d(source.x, source.y, source.z);
        if (!transformed.allFinite()) {
            continue;
        }

        PointT point;
        point.x = static_cast<float>(transformed.x());
        point.y = static_cast<float>(transformed.y());
        point.z = static_cast<float>(transformed.z());
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
    output.header.stamp = pending.received_ros_time;
    output.header.frame_id = target_frame_;
    cloud_publisher_->publish(output);
    return true;
}

} // namespace dart_vision::lidar
