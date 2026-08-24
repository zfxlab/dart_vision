#ifndef DART_VISION_LIDAR_LOCALIZATION_CORE_ALIGNMENT_METRICS_HPP
#define DART_VISION_LIDAR_LOCALIZATION_CORE_ALIGNMENT_METRICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <pcl/kdtree/kdtree_flann.h>
#include <string_view>

#include "dart_vision_lidar_localization/core/cloud_utils.hpp"

namespace dart_vision::lidar {

// All transforms in this package use the convention t_target_source: a point
// expressed in source is mapped into target by p_target = t_target_source * p_source.
enum class LocalizationStatus : std::uint8_t {
    kValid = 0,
    kMissingTemplate,
    kInsufficientScenePoints,
    kInvalidConfiguration,
    kNonFiniteTransform,
    kIcpNotConverged,
    kTranslationBoundExceeded,
    kRotationBoundExceeded,
    kResidualTooHigh,
    kInlierRatioTooLow,
    kCoverageTooLow,
    kScoreTooLow,
    kAmbiguousState,
    kTemporalUnconfirmed,
    kTemporalDiscontinuity,
    kNoSearchCandidate,
};

std::string_view toString(LocalizationStatus status) noexcept;

struct AlignmentMetricConfig {
    // Nearest-neighbor distances larger than this value contribute exactly this
    // value to the truncated RMSE, limiting the influence of residual background.
    double nearest_neighbor_truncation_m{0.12};
    double inlier_distance_m{0.05};
    double max_truncated_rmse_m{0.075};
    double min_inlier_ratio{0.35};
    // Coverage is the fraction of template samples which receive at least one
    // inlier from the scene->template correspondence pass. It is not a reverse
    // template->scene registration objective, so unobserved CAD is never forced
    // to find a scene correspondence.
    double min_template_coverage_ratio{0.05};
    double min_score{0.45};
    std::size_t min_scene_points{20};

    double residual_score_weight{0.45};
    double inlier_score_weight{0.35};
    double coverage_score_weight{0.20};
};

struct AlignmentMetrics {
    double truncated_rmse_m{std::numeric_limits<double>::infinity()};
    double inlier_ratio{0.0};
    double template_coverage_ratio{0.0};
    double score{0.0};

    std::size_t scene_points{0};
    std::size_t inlier_points{0};
    std::size_t template_points{0};
    std::size_t covered_template_points{0};
};

struct AlignmentEvaluation {
    AlignmentMetrics metrics{};
    bool valid{false};
    LocalizationStatus status{LocalizationStatus::kInsufficientScenePoints};
};

// Caches a finite template and its nearest-neighbor index. Scene points are
// transformed into the template frame and queried in one direction only.
class TemplateAlignmentEvaluator {
public:
    TemplateAlignmentEvaluator();
    explicit TemplateAlignmentEvaluator(const PointCloud::ConstPtr& template_cloud);

    void setTemplate(const PointCloud::ConstPtr& template_cloud);
    [[nodiscard]] bool hasTemplate() const noexcept;
    [[nodiscard]] const PointCloud::ConstPtr& templateCloud() const noexcept;

    [[nodiscard]] AlignmentEvaluation evaluate(const PointCloud& scene_registration,
                                               const Eigen::Isometry3d& t_registration_template,
                                               const AlignmentMetricConfig& config) const;

private:
    PointCloud::ConstPtr template_cloud_;
    pcl::KdTreeFLANN<PointT> nearest_neighbor_tree_;
};

[[nodiscard]] bool isFiniteTransform(const Eigen::Isometry3d& transform) noexcept;

// Returns the geodesic SO(3) angle in [0, pi].
[[nodiscard]] double rotationAngle(const Eigen::Matrix3d& rotation) noexcept;

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_LOCALIZATION_CORE_ALIGNMENT_METRICS_HPP
