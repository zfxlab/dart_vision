#ifndef DART_VISION_LIDAR_LOCALIZATION_LOCALIZERS_ARMOR_LOCALIZER_HPP
#define DART_VISION_LIDAR_LOCALIZATION_LOCALIZERS_ARMOR_LOCALIZER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>

#include "dart_vision_lidar_localization/core/alignment_metrics.hpp"
#include "dart_vision_lidar_localization/core/temporal_gate.hpp"

namespace dart_vision::lidar {

struct ArmorLocalizerConfig {
    AlignmentMetricConfig metrics{};

    // Axis is expressed in base coordinates and normalized internally.
    Eigen::Vector3d motion_axis_base{Eigen::Vector3d::UnitX()};
    double min_axis_position_m{0.0};
    double max_axis_position_m{1.0};
    double coarse_step_m{0.05};
    double fine_step_m{0.005};
    double fine_half_window_m{0.05};
    // Hard protection against accidental sub-micrometre steps over a long axis.
    std::size_t max_search_candidates{10000U};
    // A fixed base-frame AABB enclosing the entire configured axis sweep is
    // expanded by this amount before candidate evaluation.
    double sweep_roi_padding_m{0.08};
    TemporalGateConfig temporal{};
};

struct ArmorSearchCandidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    LocalizationStatus status{LocalizationStatus::kNoSearchCandidate};
    double axis_position_m{0.0};
    Eigen::Isometry3d t_registration_armor{Eigen::Isometry3d::Identity()};
    AlignmentMetrics metrics{};
};

struct ArmorLocalizationResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    LocalizationStatus status{LocalizationStatus::kTemporalUnconfirmed};
    double confidence{0.0};
    double axis_position_m{0.0};
    std::size_t consecutive_frames{0};
    std::size_t evaluated_candidates{0};
    std::size_t roi_scene_points{0};
    Eigen::Vector3d sweep_roi_min_base_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d sweep_roi_max_base_m{Eigen::Vector3d::Zero()};
    Eigen::Isometry3d t_registration_armor{Eigen::Isometry3d::Identity()};
    ArmorSearchCandidate best_candidate{};
};

class ArmorLocalizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit ArmorLocalizer(ArmorLocalizerConfig config = {});
    ArmorLocalizer(const PointCloud::ConstPtr& armor_template,
                   const Eigen::Isometry3d& t_base_armor_zero,
                   ArmorLocalizerConfig config = {});

    void setTemplate(const PointCloud::ConstPtr& armor_template);
    void setZeroPose(const Eigen::Isometry3d& t_base_armor_zero);
    void setConfig(const ArmorLocalizerConfig& config);
    [[nodiscard]] const ArmorLocalizerConfig& config() const noexcept;
    [[nodiscard]] const Eigen::Isometry3d& zeroPose() const noexcept;
    void reset() noexcept;

    // The only estimated DoF is s. Every candidate is exactly
    // t_registration_base * Translation(normalized_axis_base * s) * t_base_armor_zero.
    [[nodiscard]] ArmorLocalizationResult localize(const PointCloud& scene_registration,
                                                   const Eigen::Isometry3d& t_registration_base);

private:
    [[nodiscard]] bool configIsValid() const noexcept;
    [[nodiscard]] bool computeSweepRoi(Eigen::Vector3d& minimum_base_m,
                                       Eigen::Vector3d& maximum_base_m) const noexcept;
    [[nodiscard]] PointCloud cropToSweepRoi(const PointCloud& scene_registration,
                                            const Eigen::Isometry3d& t_registration_base,
                                            const Eigen::Vector3d& minimum_base_m,
                                            const Eigen::Vector3d& maximum_base_m) const;
    [[nodiscard]] ArmorSearchCandidate
    evaluatePosition(const PointCloud& scene_registration,
                     const Eigen::Isometry3d& t_registration_base,
                     double axis_position_m) const;

    ArmorLocalizerConfig config_{};
    Eigen::Isometry3d t_base_armor_zero_{Eigen::Isometry3d::Identity()};
    TemplateAlignmentEvaluator evaluator_{};
    TemporalGate temporal_gate_{};
};

struct GreenLightEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    Eigen::Vector3d position_registration_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d position_output_m{Eigen::Vector3d::Zero()};
    double distance_from_registration_origin_m{0.0};
    double distance_from_output_origin_m{0.0};
};

// offset_armor_m is the fixed green-light position expressed in the armor frame.
// t_output_registration maps registration-frame coordinates to the requested output frame.
[[nodiscard]] GreenLightEstimate computeGreenLightEstimate(
    const Eigen::Isometry3d& t_registration_armor,
    const Eigen::Vector3d& offset_armor_m,
    const Eigen::Isometry3d& t_output_registration = Eigen::Isometry3d::Identity());

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_LOCALIZATION_LOCALIZERS_ARMOR_LOCALIZER_HPP
