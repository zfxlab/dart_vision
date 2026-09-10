#ifndef DART_LIDAR_LOCALIZATION_LOCALIZATION_TYPES_HPP
#define DART_LIDAR_LOCALIZATION_LOCALIZATION_TYPES_HPP

#include <Eigen/Geometry>
#include <cstddef>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace dart_vision::lidar::localization {

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

enum class BaseRegistrationStatus {
    kSuccess = 0,
    kInvalidInput = 1,
    kCoarseFailed = 2,
    kFineFailed = 3,
    kQualityRejected = 4,
    kTimeBudgetExceeded = 5,
    kInternalError = 6,
    kTransformUnavailable = 7,
};

enum class DofMode { kSixDof, kXyzYaw, kXyYaw };

struct NdtLevelParameters {
    double voxel_leaf_size_m{0.10};
    double resolution_m{0.12};
    double step_size_m{0.10};
    double transformation_epsilon{1.0e-3};
    int max_iterations{60};
};

struct GicpLevelParameters {
    double voxel_leaf_size_m{0.03};
    double max_correspondence_distance_m{0.08};
    double transformation_epsilon{1.0e-5};
    double fitness_epsilon{1.0e-5};
    int max_iterations{20};
};

struct BaseValidationParameters {
    double correspondence_distance_m{0.05};
    double max_rmse_m{0.025};
    double min_overlap_ratio{0.35};
    std::size_t min_correspondences{500U};
    double max_translation_from_initial_m{0.08};
    double max_yaw_from_initial_rad{0.035};
    // 默认只做点云质量验收；如需在LiDAR层限制相对初值的偏差再显式启用。
    bool enforce_initial_deviation_limits{false};
    bool reject_if_search_boundary_hit{true};
    double boundary_ratio{0.98};
};

struct BaseRegistrationParameters {
    bool coarse_enabled{false};
    std::vector<NdtLevelParameters> ndt_levels;
    std::vector<GicpLevelParameters> gicp_levels;
    DofMode dof_mode{DofMode::kXyzYaw};
    BaseValidationParameters validation;
    double max_total_time_ms{250.0};
    std::size_t min_input_points{100U};
};

struct BaseRegistrationMetrics {
    double rmse_m{0.0};
    double overlap_ratio{0.0};
    std::size_t correspondence_count{0U};
    double translation_from_initial_m{0.0};
    double yaw_from_initial_rad{0.0};
    bool search_boundary_hit{false};
};

struct BaseStageResult {
    bool converged{false};
    Eigen::Isometry3d target_from_source{Eigen::Isometry3d::Identity()};
    int iterations{0};
    double fitness_score{0.0};
};

struct BaseRegistrationResult {
    BaseRegistrationStatus status{BaseRegistrationStatus::kInternalError};
    std::string message;
    Eigen::Isometry3d target_from_source{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d observation_from_model{Eigen::Isometry3d::Identity()};
    BaseRegistrationMetrics metrics;
    int total_iterations{0};
    double elapsed_ms{0.0};
    bool coarse_used{false};

    [[nodiscard]] bool success() const noexcept {
        return status == BaseRegistrationStatus::kSuccess;
    }

    // 质量拒绝发生在完整配准之后，其变换仍可用于调试可视化。
    [[nodiscard]] bool hasCandidate() const noexcept {
        return success() || status == BaseRegistrationStatus::kQualityRejected;
    }
};

struct ModuleLocalizationParameters {
    double min_position_m{0.0};
    double max_position_m{0.56};
    double coarse_step_m{0.005};
    double fine_step_m{0.0005};
    double fine_half_window_m{0.02};
    double max_correspondence_distance_m{0.025};
    double max_rmse_m{0.015};
    double min_overlap_ratio{0.30};
    std::size_t min_correspondences{10U};
    Eigen::Vector3d roi_padding_m{0.03, 0.03, 0.03};
    double ambiguity_separation_m{0.03};
    double min_objective_gap_m{0.001};
};

struct ModuleLocalizationMetrics {
    double rmse_m{0.0};
    double overlap_ratio{0.0};
    std::size_t correspondence_count{0U};
    bool search_boundary_hit{false};
};

struct ModuleLocalizationResult {
    bool available{false};
    bool has_candidate{false};
    std::string message;
    double position_m{0.0};
    ModuleLocalizationMetrics metrics;
};

[[nodiscard]] const char* baseStatusMessage(BaseRegistrationStatus status) noexcept;
[[nodiscard]] DofMode parseDofMode(const std::string& value);

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_LOCALIZATION_TYPES_HPP
