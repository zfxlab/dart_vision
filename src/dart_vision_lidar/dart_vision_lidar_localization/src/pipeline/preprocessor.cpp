#include "dart_vision_lidar_localization/pipeline/preprocessor.hpp"

#include <cmath>
#include <memory>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <stdexcept>
#include <utility>

namespace dart_vision::lidar
{
namespace
{

PointCloud::Ptr makeCloudWithMetadataFrom(const PointCloud & source)
{
  PointCloud::Ptr cloud = std::make_shared<PointCloud>();
  cloud->header = source.header;
  cloud->sensor_origin_ = source.sensor_origin_;
  cloud->sensor_orientation_ = source.sensor_orientation_;
  return cloud;
}

void copySpatialMetadata(const PointCloud & source, PointCloud & target)
{
  target.header = source.header;
  target.sensor_origin_ = source.sensor_origin_;
  target.sensor_orientation_ = source.sensor_orientation_;
  finalizePointCloudMetadata(target);
}

bool passesDistanceAndCropBox(
  const PointT & point,
  const PointCloudPreprocessorConfig & config) noexcept
{
  if (!isPointFinite(point)) {
    return false;
  }

  const double x_m = static_cast<double>(point.x);
  const double y_m = static_cast<double>(point.y);
  const double z_m = static_cast<double>(point.z);
  const double distance_squared = x_m * x_m + y_m * y_m + z_m * z_m;
  const double min_distance_squared = config.min_distance_m * config.min_distance_m;
  const double max_distance_squared = config.max_distance_m * config.max_distance_m;
  if (distance_squared < min_distance_squared || distance_squared > max_distance_squared) {
    return false;
  }

  if (!config.crop_box_enabled) {
    return true;
  }

  const bool inside =
    point.x >= config.crop_box_min_m.x() && point.x <= config.crop_box_max_m.x() &&
    point.y >= config.crop_box_min_m.y() && point.y <= config.crop_box_max_m.y() &&
    point.z >= config.crop_box_min_m.z() && point.z <= config.crop_box_max_m.z();
  return config.crop_box_negative ? !inside : inside;
}

} // namespace

bool PointCloudPreprocessorConfig::isValid() const noexcept
{
  if (!std::isfinite(min_distance_m) || min_distance_m < 0.0 || std::isnan(max_distance_m) ||
    max_distance_m < min_distance_m)
  {
    return false;
  }

  if (crop_box_enabled && (!crop_box_min_m.allFinite() || !crop_box_max_m.allFinite() ||
    (crop_box_min_m.array() > crop_box_max_m.array()).any()))
  {
    return false;
  }

  if (voxel_grid_enabled &&
    (!voxel_leaf_size_m.allFinite() || (voxel_leaf_size_m.array() <= 0.0F).any()))
  {
    return false;
  }

  if (statistical_outlier_removal_enabled &&
    (sor_mean_k < 2 || !std::isfinite(sor_stddev_mul_threshold) ||
    sor_stddev_mul_threshold <= 0.0))
  {
    return false;
  }

  return true;
}

PointCloudPreprocessor::PointCloudPreprocessor(PointCloudPreprocessorConfig config)
: config_(std::move(config))
{
  if (!config_.isValid()) {
    throw std::invalid_argument("Invalid metric point cloud preprocessing configuration.");
  }
}

PointCloud PointCloudPreprocessor::process(
  const PointCloud & input,
  PointCloudPreprocessingStats * stats) const
{
  PointCloudPreprocessingStats local_stats;
  local_stats.input_points = input.size();

  PointCloud::Ptr current = makeCloudWithMetadataFrom(input);
  current->points.reserve(input.size());
  for (const PointT & point : input.points) {
    if (isPointFinite(point)) {
      current->points.push_back(point);
    }
  }
  finalizePointCloudMetadata(*current);
  local_stats.points_after_nan_removal = current->size();

  PointCloud::Ptr distance_clipped = makeCloudWithMetadataFrom(*current);
  distance_clipped->points.reserve(current->size());
  const double min_distance_squared = config_.min_distance_m * config_.min_distance_m;
  const double max_distance_squared = config_.max_distance_m * config_.max_distance_m;
  for (const PointT & point : current->points) {
    const double x_m = static_cast<double>(point.x);
    const double y_m = static_cast<double>(point.y);
    const double z_m = static_cast<double>(point.z);
    const double distance_squared = x_m * x_m + y_m * y_m + z_m * z_m;
    if (distance_squared >= min_distance_squared && distance_squared <= max_distance_squared) {
      distance_clipped->points.push_back(point);
    }
  }
  finalizePointCloudMetadata(*distance_clipped);
  current = std::move(distance_clipped);
  local_stats.points_after_distance_clip = current->size();

  if (config_.crop_box_enabled && !current->empty()) {
    PointCloud::Ptr cropped = makeCloudWithMetadataFrom(*current);
    pcl::CropBox<PointT> crop_box;
    crop_box.setInputCloud(current);
    crop_box.setMin(
      Eigen::Vector4f{config_.crop_box_min_m.x(),
        config_.crop_box_min_m.y(),
        config_.crop_box_min_m.z(),
        1.0F});
    crop_box.setMax(
      Eigen::Vector4f{config_.crop_box_max_m.x(),
        config_.crop_box_max_m.y(),
        config_.crop_box_max_m.z(),
        1.0F});
    crop_box.setNegative(config_.crop_box_negative);
    crop_box.filter(*cropped);
    copySpatialMetadata(*current, *cropped);
    current = std::move(cropped);
  }
  local_stats.points_after_crop_box = current->size();

  if (config_.voxel_grid_enabled && !current->empty()) {
    PointCloud::Ptr downsampled = makeCloudWithMetadataFrom(*current);
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(current);
    voxel_grid.setLeafSize(
      config_.voxel_leaf_size_m.x(),
      config_.voxel_leaf_size_m.y(),
      config_.voxel_leaf_size_m.z());
    voxel_grid.filter(*downsampled);
    copySpatialMetadata(*current, *downsampled);
    current = std::move(downsampled);
  }
  local_stats.points_after_voxel_grid = current->size();

  if (config_.statistical_outlier_removal_enabled &&
    current->size() > static_cast<std::size_t>(config_.sor_mean_k))
  {
    PointCloud::Ptr filtered = makeCloudWithMetadataFrom(*current);
    pcl::StatisticalOutlierRemoval<PointT> outlier_removal;
    outlier_removal.setInputCloud(current);
    outlier_removal.setMeanK(config_.sor_mean_k);
    outlier_removal.setStddevMulThresh(config_.sor_stddev_mul_threshold);
    outlier_removal.filter(*filtered);
    copySpatialMetadata(*current, *filtered);
    current = std::move(filtered);
    local_stats.statistical_outlier_removal_applied = true;
  }

  local_stats.output_points = current->size();
  if (stats != nullptr) {
    *stats = local_stats;
  }
  return *current;
}

bool PointCloudPreprocessor::hasUsablePoint(const PointCloud & input) const noexcept
{
  for (const PointT & point : input.points) {
    if (passesDistanceAndCropBox(point, config_)) {
      return true;
    }
  }
  return false;
}

const PointCloudPreprocessorConfig & PointCloudPreprocessor::config() const noexcept
{
  return config_;
}

} // namespace dart_vision::lidar
