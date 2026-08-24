#ifndef DART_VISION_LIDAR_LOCALIZATION_LOCALIZERS_BASE_LOCALIZER_HPP
#define DART_VISION_LIDAR_LOCALIZATION_LOCALIZERS_BASE_LOCALIZER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>

#include "dart_vision_lidar_localization/core/alignment_metrics.hpp"
#include "dart_vision_lidar_localization/core/temporal_gate.hpp"

namespace dart_vision::lidar {

enum class BaseState : std::int8_t {
    kUnknown = -1,
    kOpen = 0,
    kClosed = 1,
};

struct BaseLocalizerConfig {
    AlignmentMetricConfig metrics{};
    int icp_max_iterations{40};
    double icp_max_correspondence_distance_m{0.12};
    double icp_transformation_epsilon{1.0e-8};
    double icp_euclidean_fitness_epsilon{1.0e-7};

    // ICP is constrained to x/y/yaw in the initial base frame. Base height and
    // its z-axis direction therefore remain fixed. In this constrained result,
    // max_rotation_delta_rad is the maximum allowed yaw correction.
    double max_translation_delta_m{0.25};
    double max_rotation_delta_rad{0.35};
    bool require_icp_convergence{true};
    bool parallel_candidates{true};

    double min_state_score_margin{0.05};
    TemporalGateConfig temporal{};
};

struct BaseCandidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BaseState state{BaseState::kUnknown};
    bool valid{false};
    bool optimizer_converged{false};
    LocalizationStatus status{LocalizationStatus::kMissingTemplate};
    Eigen::Isometry3d t_registration_base{Eigen::Isometry3d::Identity()};
    double translation_delta_m{0.0};
    double rotation_delta_rad{0.0};
    double icp_fitness_score{0.0};
    AlignmentMetrics metrics{};
};

struct BaseLocalizationResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    BaseState state{BaseState::kUnknown};
    LocalizationStatus status{LocalizationStatus::kTemporalUnconfirmed};
    double confidence{0.0};
    double state_score_margin{0.0};
    std::size_t consecutive_frames{0};
    Eigen::Isometry3d t_registration_base{Eigen::Isometry3d::Identity()};
    BaseCandidate open_candidate{};
    BaseCandidate closed_candidate{};
};

class BaseLocalizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit BaseLocalizer(BaseLocalizerConfig config = {});
    BaseLocalizer(const PointCloud::ConstPtr& open_template,
                  const PointCloud::ConstPtr& closed_template,
                  BaseLocalizerConfig config = {});

    void setTemplates(const PointCloud::ConstPtr& open_template,
                      const PointCloud::ConstPtr& closed_template);
    void setConfig(const BaseLocalizerConfig& config);
    [[nodiscard]] const BaseLocalizerConfig& config() const noexcept;
    void reset() noexcept;

    // scene_registration is expressed in the registration frame. The returned
    // t_registration_base maps base-template points into that same frame.
    [[nodiscard]] BaseLocalizationResult
    localize(const PointCloud& scene_registration,
             const Eigen::Isometry3d& t_registration_base_initial);

private:
    void updateTemplateBounds() noexcept;
    [[nodiscard]] bool configIsValid() const noexcept;
    [[nodiscard]] PointCloud::ConstPtr
    cropSceneToSearchRegion(const PointCloud& scene_registration,
                            const Eigen::Isometry3d& t_registration_base_initial) const;
    [[nodiscard]] BaseCandidate alignCandidate(const PointCloud::ConstPtr& scene_registration,
                                               const Eigen::Isometry3d& t_registration_base_initial,
                                               BaseState state,
                                               const TemplateAlignmentEvaluator& evaluator) const;

    BaseLocalizerConfig config_{};
    TemplateAlignmentEvaluator open_evaluator_{};
    TemplateAlignmentEvaluator closed_evaluator_{};
    TemporalGate temporal_gate_{};
    Eigen::Vector3d template_minimum_base_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d template_maximum_base_{Eigen::Vector3d::Zero()};
    double template_maximum_radius_base_{0.0};
    bool template_bounds_valid_{false};
};

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_LOCALIZATION_LOCALIZERS_BASE_LOCALIZER_HPP
