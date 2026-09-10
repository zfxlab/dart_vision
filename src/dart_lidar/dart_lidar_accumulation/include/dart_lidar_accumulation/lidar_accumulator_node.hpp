#ifndef DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_NODE_HPP
#define DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_NODE_HPP

#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "dart_lidar_accumulation/lidar_accumulator_config.hpp"

namespace dart_vision::lidar {

class LidarAccumulatorNode : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<PointT>;

    explicit LidarAccumulatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void declareParameters();
    [[nodiscard]] LidarAccumulatorConfig readConfig() const;
    [[nodiscard]] Eigen::Vector3f readVector3(const std::string& name) const;
    [[nodiscard]] rcl_interfaces::msg::SetParametersResult
    onParametersChanged(const std::vector<rclcpp::Parameter>& parameters);

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
    [[nodiscard]] PointCloud::Ptr cropCloud(const sensor_msgs::msg::PointCloud2& message) const;
    void addFrame(PointCloud::Ptr cloud,
                  std::int64_t stamp_ns,
                  const builtin_interfaces::msg::Time& stamp);
    void pruneWindow(std::int64_t newest_stamp_ns);
    [[nodiscard]] bool publicationDue(std::int64_t newest_stamp_ns) const;
    void publishWindow();
    void resetWindow();

    struct StampedCloud {
        PointCloud::Ptr cloud;
        std::int64_t stamp_ns{0};
        builtin_interfaces::msg::Time stamp;
    };

    mutable std::mutex config_mutex_;
    LidarAccumulatorConfig config_;

    std::int64_t last_stamp_ns_{0};
    std::int64_t last_publish_stamp_ns_{0};
    std::deque<StampedCloud> frames_;
    std::size_t accumulated_points_{0U};
    std::size_t new_frames_since_publish_{0U};
    bool has_published_{false};

    std::uint64_t received_clouds_{0U};
    std::uint64_t accepted_clouds_{0U};
    std::uint64_t dropped_clouds_{0U};
    std::uint64_t published_clouds_{0U};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

} // namespace dart_vision::lidar

#endif // DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_NODE_HPP
