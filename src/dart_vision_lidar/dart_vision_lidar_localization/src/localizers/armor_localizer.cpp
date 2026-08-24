#include "dart_vision_lidar_localization/localizers/armor_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace dart_vision::lidar {
namespace {

std::vector<double>
uniformlySpacedSamples(const double minimum, const double maximum, const double step) {
    std::vector<double> samples;
    if (!(minimum <= maximum) || !(step > 0.0)) {
        return samples;
    }
    const double span = maximum - minimum;
    const std::size_t whole_steps = static_cast<std::size_t>(std::floor(span / step + 1.0e-9));
    samples.reserve(whole_steps + 2U);
    for (std::size_t index = 0; index <= whole_steps; ++index) {
        samples.push_back(minimum + static_cast<double>(index) * step);
    }
    const double endpoint_tolerance = std::max(1.0, std::abs(maximum)) * 1.0e-12;
    if (samples.empty() || maximum - samples.back() > endpoint_tolerance) {
        samples.push_back(maximum);
    }
    return samples;
}

bool betterCandidate(const ArmorSearchCandidate& candidate,
                     const ArmorSearchCandidate& current_best,
                     const bool have_best) {
    if (!have_best) {
        return true;
    }
    if (candidate.valid != current_best.valid) {
        return candidate.valid;
    }
    return candidate.metrics.score > current_best.metrics.score;
}

} // namespace

ArmorLocalizer::ArmorLocalizer(ArmorLocalizerConfig config)
    : config_(config), temporal_gate_(config.temporal) {}

ArmorLocalizer::ArmorLocalizer(const PointCloud::ConstPtr& armor_template,
                               const Eigen::Isometry3d& t_base_armor_zero,
                               ArmorLocalizerConfig config)
    : config_(config), t_base_armor_zero_(t_base_armor_zero), evaluator_(armor_template),
      temporal_gate_(config.temporal) {}

void ArmorLocalizer::setTemplate(const PointCloud::ConstPtr& armor_template) {
    evaluator_.setTemplate(armor_template);
    reset();
}

void ArmorLocalizer::setZeroPose(const Eigen::Isometry3d& t_base_armor_zero) {
    t_base_armor_zero_ = t_base_armor_zero;
    reset();
}

void ArmorLocalizer::setConfig(const ArmorLocalizerConfig& config) {
    config_ = config;
    temporal_gate_.setConfig(config.temporal);
}

const ArmorLocalizerConfig& ArmorLocalizer::config() const noexcept {
    return config_;
}

const Eigen::Isometry3d& ArmorLocalizer::zeroPose() const noexcept {
    return t_base_armor_zero_;
}

void ArmorLocalizer::reset() noexcept {
    temporal_gate_.reset();
}

ArmorLocalizationResult ArmorLocalizer::localize(const PointCloud& scene_registration,
                                                 const Eigen::Isometry3d& t_registration_base) {
    ArmorLocalizationResult result;
    if (!configIsValid()) {
        result.status = LocalizationStatus::kInvalidConfiguration;
        temporal_gate_.reset();
        return result;
    }
    if (!evaluator_.hasTemplate()) {
        result.status = LocalizationStatus::kMissingTemplate;
        temporal_gate_.reset();
        return result;
    }
    if (!isFiniteTransform(t_registration_base) || !isFiniteTransform(t_base_armor_zero_)) {
        result.status = LocalizationStatus::kNonFiniteTransform;
        temporal_gate_.reset();
        return result;
    }
    if (!computeSweepRoi(result.sweep_roi_min_base_m, result.sweep_roi_max_base_m)) {
        result.status = LocalizationStatus::kInvalidConfiguration;
        temporal_gate_.reset();
        return result;
    }

    // Candidate metrics must see the same scene support. Cropping once to the
    // full base-frame sweep volume avoids candidate-dependent ROI score bias.
    const PointCloud armor_scene_registration = cropToSweepRoi(scene_registration,
                                                               t_registration_base,
                                                               result.sweep_roi_min_base_m,
                                                               result.sweep_roi_max_base_m);
    result.roi_scene_points = armor_scene_registration.size();

    bool have_coarse_best = false;
    ArmorSearchCandidate coarse_best;
    const auto coarse_positions = uniformlySpacedSamples(
        config_.min_axis_position_m, config_.max_axis_position_m, config_.coarse_step_m);
    for (const double position : coarse_positions) {
        const ArmorSearchCandidate candidate =
            evaluatePosition(armor_scene_registration, t_registration_base, position);
        ++result.evaluated_candidates;
        if (betterCandidate(candidate, coarse_best, have_coarse_best)) {
            coarse_best = candidate;
            have_coarse_best = true;
        }
    }

    if (!have_coarse_best) {
        result.status = LocalizationStatus::kNoSearchCandidate;
        temporal_gate_.reset();
        return result;
    }

    const double fine_minimum = std::max(config_.min_axis_position_m,
                                         coarse_best.axis_position_m - config_.fine_half_window_m);
    const double fine_maximum = std::min(config_.max_axis_position_m,
                                         coarse_best.axis_position_m + config_.fine_half_window_m);
    bool have_fine_best = false;
    ArmorSearchCandidate fine_best;
    const auto fine_positions =
        uniformlySpacedSamples(fine_minimum, fine_maximum, config_.fine_step_m);
    for (const double position : fine_positions) {
        const ArmorSearchCandidate candidate =
            evaluatePosition(armor_scene_registration, t_registration_base, position);
        ++result.evaluated_candidates;
        if (betterCandidate(candidate, fine_best, have_fine_best)) {
            fine_best = candidate;
            have_fine_best = true;
        }
    }

    result.best_candidate = coarse_best;
    if (have_fine_best && betterCandidate(fine_best, result.best_candidate, true)) {
        result.best_candidate = fine_best;
    }
    result.axis_position_m = result.best_candidate.axis_position_m;
    result.t_registration_armor = result.best_candidate.t_registration_armor;
    result.confidence = result.best_candidate.metrics.score;

    TemporalGateInput gate_input;
    gate_input.valid = result.best_candidate.valid;
    gate_input.state = 0;
    gate_input.pose = result.t_registration_armor;
    gate_input.confidence = result.confidence;
    gate_input.status = result.best_candidate.status;
    const TemporalGateOutput gate_output = temporal_gate_.update(gate_input);
    result.valid = gate_output.valid;
    result.status = gate_output.status;
    result.consecutive_frames = gate_output.consecutive_frames;
    return result;
}

bool ArmorLocalizer::configIsValid() const noexcept {
    if (!(config_.motion_axis_base.allFinite() && config_.motion_axis_base.norm() > 1.0e-12 &&
          std::isfinite(config_.min_axis_position_m) &&
          std::isfinite(config_.max_axis_position_m) &&
          config_.min_axis_position_m <= config_.max_axis_position_m &&
          std::isfinite(config_.coarse_step_m) && config_.coarse_step_m > 0.0 &&
          std::isfinite(config_.fine_step_m) && config_.fine_step_m > 0.0 &&
          std::isfinite(config_.fine_half_window_m) && config_.fine_half_window_m >= 0.0 &&
          std::isfinite(config_.sweep_roi_padding_m) && config_.sweep_roi_padding_m >= 0.0 &&
          config_.max_search_candidates > 0U)) {
        return false;
    }
    const double coarse_count =
        (config_.max_axis_position_m - config_.min_axis_position_m) / config_.coarse_step_m + 2.0;
    const double fine_count = 2.0 * config_.fine_half_window_m / config_.fine_step_m + 2.0;
    return std::isfinite(coarse_count) && std::isfinite(fine_count) &&
           coarse_count + fine_count <= static_cast<double>(config_.max_search_candidates);
}

bool ArmorLocalizer::computeSweepRoi(Eigen::Vector3d& minimum_base_m,
                                     Eigen::Vector3d& maximum_base_m) const noexcept {
    if (!evaluator_.hasTemplate() || !isFiniteTransform(t_base_armor_zero_)) {
        return false;
    }
    minimum_base_m = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    maximum_base_m = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    const Eigen::Vector3d axis_base = config_.motion_axis_base.normalized();
    for (const PointT& point_armor : *evaluator_.templateCloud()) {
        const Eigen::Vector3d point_zero_base =
            t_base_armor_zero_ * Eigen::Vector3d(point_armor.x, point_armor.y, point_armor.z);
        const Eigen::Vector3d point_at_minimum =
            point_zero_base + axis_base * config_.min_axis_position_m;
        const Eigen::Vector3d point_at_maximum =
            point_zero_base + axis_base * config_.max_axis_position_m;
        minimum_base_m = minimum_base_m.cwiseMin(point_at_minimum).cwiseMin(point_at_maximum);
        maximum_base_m = maximum_base_m.cwiseMax(point_at_minimum).cwiseMax(point_at_maximum);
    }
    minimum_base_m.array() -= config_.sweep_roi_padding_m;
    maximum_base_m.array() += config_.sweep_roi_padding_m;
    return minimum_base_m.allFinite() && maximum_base_m.allFinite() &&
           (minimum_base_m.array() <= maximum_base_m.array()).all();
}

PointCloud ArmorLocalizer::cropToSweepRoi(const PointCloud& scene_registration,
                                          const Eigen::Isometry3d& t_registration_base,
                                          const Eigen::Vector3d& minimum_base_m,
                                          const Eigen::Vector3d& maximum_base_m) const {
    PointCloud cropped;
    cropped.header = scene_registration.header;
    cropped.sensor_origin_ = scene_registration.sensor_origin_;
    cropped.sensor_orientation_ = scene_registration.sensor_orientation_;
    cropped.reserve(scene_registration.size());
    const Eigen::Isometry3d t_base_registration = t_registration_base.inverse();
    for (const PointT& point_registration : scene_registration) {
        if (!isPointFinite(point_registration)) {
            continue;
        }
        const Eigen::Vector3d point_base =
            t_base_registration *
            Eigen::Vector3d(point_registration.x, point_registration.y, point_registration.z);
        if ((point_base.array() >= minimum_base_m.array()).all() &&
            (point_base.array() <= maximum_base_m.array()).all()) {
            cropped.push_back(point_registration);
        }
    }
    finalizePointCloudMetadata(cropped);
    return cropped;
}

ArmorSearchCandidate ArmorLocalizer::evaluatePosition(const PointCloud& scene_registration,
                                                      const Eigen::Isometry3d& t_registration_base,
                                                      const double axis_position_m) const {
    ArmorSearchCandidate candidate;
    candidate.axis_position_m = axis_position_m;
    const Eigen::Vector3d normalized_axis = config_.motion_axis_base.normalized();
    const Eigen::Isometry3d t_base_armor =
        Eigen::Translation3d(normalized_axis * axis_position_m) * t_base_armor_zero_;
    candidate.t_registration_armor = t_registration_base * t_base_armor;
    const AlignmentEvaluation evaluation =
        evaluator_.evaluate(scene_registration, candidate.t_registration_armor, config_.metrics);
    candidate.metrics = evaluation.metrics;
    candidate.valid = evaluation.valid;
    candidate.status = evaluation.status;
    return candidate;
}

GreenLightEstimate computeGreenLightEstimate(const Eigen::Isometry3d& t_registration_armor,
                                             const Eigen::Vector3d& offset_armor_m,
                                             const Eigen::Isometry3d& t_output_registration) {
    GreenLightEstimate estimate;
    if (!isFiniteTransform(t_registration_armor) || !isFiniteTransform(t_output_registration) ||
        !offset_armor_m.allFinite()) {
        return estimate;
    }
    estimate.position_registration_m = t_registration_armor * offset_armor_m;
    estimate.position_output_m = t_output_registration * estimate.position_registration_m;
    estimate.distance_from_registration_origin_m = estimate.position_registration_m.norm();
    estimate.distance_from_output_origin_m = estimate.position_output_m.norm();
    estimate.valid = estimate.position_registration_m.allFinite() &&
                     estimate.position_output_m.allFinite() &&
                     std::isfinite(estimate.distance_from_registration_origin_m) &&
                     std::isfinite(estimate.distance_from_output_origin_m);
    return estimate;
}

} // namespace dart_vision::lidar
