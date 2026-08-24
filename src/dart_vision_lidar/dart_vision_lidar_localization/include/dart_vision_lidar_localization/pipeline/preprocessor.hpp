#ifndef DART_VISION_LIDAR_LOCALIZATION_PIPELINE_PREPROCESSOR_HPP
#define DART_VISION_LIDAR_LOCALIZATION_PIPELINE_PREPROCESSOR_HPP

#include <Eigen/Core>
#include <cstddef>
#include <limits>

#include "dart_vision_lidar_localization/core/cloud_utils.hpp"

namespace dart_vision::lidar {

struct PointCloudPreprocessorConfig {
    double min_distance_m{0.0};
    double max_distance_m{std::numeric_limits<double>::infinity()};

    bool crop_box_enabled{false};
    Eigen::Vector3f crop_box_min_m{-1.0F, -1.0F, -1.0F};
    Eigen::Vector3f crop_box_max_m{1.0F, 1.0F, 1.0F};
    bool crop_box_negative{false};

    bool voxel_grid_enabled{false};
    Eigen::Vector3f voxel_leaf_size_m{0.02F, 0.02F, 0.02F};

    bool statistical_outlier_removal_enabled{false};
    int sor_mean_k{20};
    double sor_stddev_mul_threshold{1.0};

    [[nodiscard]] bool isValid() const noexcept;
};

struct PointCloudPreprocessingStats {
    std::size_t input_points{0U};
    std::size_t points_after_nan_removal{0U};
    std::size_t points_after_distance_clip{0U};
    std::size_t points_after_crop_box{0U};
    std::size_t points_after_voxel_grid{0U};
    std::size_t output_points{0U};
    bool statistical_outlier_removal_applied{false};
};

/**
 * @brief Deterministic metric point cloud preprocessing pipeline.
 *
 * The processing order is invalid-XYZ removal, Euclidean range clipping,
 * optional CropBox, optional VoxelGrid, then optional StatisticalOutlierRemoval.
 * All configuration lengths are in metres.
 */
class PointCloudPreprocessor {
public:
    explicit PointCloudPreprocessor(PointCloudPreprocessorConfig config);

    [[nodiscard]] PointCloud process(const PointCloud& input,
                                     PointCloudPreprocessingStats* stats = nullptr) const;

    /**
   * @brief Return whether at least one point passes the point-wise validity filters.
   *
   * This applies finite-XYZ, distance, and CropBox checks only. VoxelGrid cannot
   * remove the last point, while StatisticalOutlierRemoval intentionally needs a
   * neighbourhood and is therefore evaluated on the accumulated cloud.
   */
    [[nodiscard]] bool hasUsablePoint(const PointCloud& input) const noexcept;

    [[nodiscard]] const PointCloudPreprocessorConfig& config() const noexcept;

private:
    PointCloudPreprocessorConfig config_;
};

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_LOCALIZATION_PIPELINE_PREPROCESSOR_HPP
