#include "dart_vision_lidar_localization/localizers/base_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <pcl/common/point_tests.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>

namespace dart_vision::lidar {
namespace {

PointCloud::Ptr finiteCopy(const PointCloud& input) {
    PointCloud::Ptr output(new PointCloud);
    output->reserve(input.size());
    for (const auto& point : input) {
        if (pcl::isFinite(point)) {
            output->push_back(point);
        }
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1U;
    output->is_dense = true;
    return output;
}

} // namespace

BaseLocalizer::BaseLocalizer(BaseLocalizerConfig config)
    : config_(config), temporal_gate_(config.temporal) {}

BaseLocalizer::BaseLocalizer(const PointCloud::ConstPtr& open_template,
                             const PointCloud::ConstPtr& closed_template,
                             BaseLocalizerConfig config)
    : config_(config), open_evaluator_(open_template), closed_evaluator_(closed_template),
      temporal_gate_(config.temporal) {
    updateTemplateBounds();
}

void BaseLocalizer::setTemplates(const PointCloud::ConstPtr& open_template,
                                 const PointCloud::ConstPtr& closed_template) {
    open_evaluator_.setTemplate(open_template);
    closed_evaluator_.setTemplate(closed_template);
    updateTemplateBounds();
    reset();
}

void BaseLocalizer::setConfig(const BaseLocalizerConfig& config) {
    config_ = config;
    temporal_gate_.setConfig(config.temporal);
}

const BaseLocalizerConfig& BaseLocalizer::config() const noexcept {
    return config_;
}

void BaseLocalizer::reset() noexcept {
    temporal_gate_.reset();
}

void BaseLocalizer::updateTemplateBounds() noexcept {
    template_minimum_base_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    template_maximum_base_ = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    template_maximum_radius_base_ = 0.0;
    const auto include_template = [this](const TemplateAlignmentEvaluator& evaluator) {
        if (!evaluator.hasTemplate()) {
            return;
        }
        for (const PointT& point : *evaluator.templateCloud()) {
            const Eigen::Vector3d position(point.x, point.y, point.z);
            template_minimum_base_ = template_minimum_base_.cwiseMin(position);
            template_maximum_base_ = template_maximum_base_.cwiseMax(position);
            template_maximum_radius_base_ =
                std::max(template_maximum_radius_base_, position.norm());
        }
    };
    include_template(open_evaluator_);
    include_template(closed_evaluator_);
    template_bounds_valid_ =
        template_minimum_base_.allFinite() && template_maximum_base_.allFinite() &&
        (template_minimum_base_.array() <= template_maximum_base_.array()).all();
}

BaseLocalizationResult
BaseLocalizer::localize(const PointCloud& scene_registration,
                        const Eigen::Isometry3d& t_registration_base_initial) {
    BaseLocalizationResult result;
    if (!open_evaluator_.hasTemplate() || !closed_evaluator_.hasTemplate()) {
        result.open_candidate.state = BaseState::kOpen;
        result.closed_candidate.state = BaseState::kClosed;
        result.open_candidate.status = LocalizationStatus::kMissingTemplate;
        result.closed_candidate.status = LocalizationStatus::kMissingTemplate;
        result.status = LocalizationStatus::kMissingTemplate;
        temporal_gate_.reset();
        return result;
    }

    const PointCloud::ConstPtr local_scene =
        cropSceneToSearchRegion(scene_registration, t_registration_base_initial);
    if (config_.parallel_candidates) {
        auto closed_future =
            std::async(std::launch::async, [this, local_scene, t_registration_base_initial]() {
                return alignCandidate(local_scene,
                                      t_registration_base_initial,
                                      BaseState::kClosed,
                                      closed_evaluator_);
            });
        result.open_candidate = alignCandidate(
            local_scene, t_registration_base_initial, BaseState::kOpen, open_evaluator_);
        result.closed_candidate = closed_future.get();
    } else {
        result.open_candidate = alignCandidate(
            local_scene, t_registration_base_initial, BaseState::kOpen, open_evaluator_);
        result.closed_candidate = alignCandidate(
            local_scene, t_registration_base_initial, BaseState::kClosed, closed_evaluator_);
    }

    const BaseCandidate* selected = nullptr;
    if (result.open_candidate.valid && result.closed_candidate.valid) {
        selected = result.open_candidate.metrics.score >= result.closed_candidate.metrics.score
                       ? &result.open_candidate
                       : &result.closed_candidate;
    } else if (result.open_candidate.valid) {
        selected = &result.open_candidate;
    } else if (result.closed_candidate.valid) {
        selected = &result.closed_candidate;
    } else {
        selected = result.open_candidate.metrics.score >= result.closed_candidate.metrics.score
                       ? &result.open_candidate
                       : &result.closed_candidate;
    }

    result.state = selected->state;
    result.t_registration_base = selected->t_registration_base;
    result.confidence = selected->metrics.score;
    result.state_score_margin =
        std::abs(result.open_candidate.metrics.score - result.closed_candidate.metrics.score);

    bool raw_valid = selected->valid;
    LocalizationStatus raw_status = selected->status;
    if (raw_valid && result.state_score_margin < config_.min_state_score_margin) {
        raw_valid = false;
        raw_status = LocalizationStatus::kAmbiguousState;
    }

    TemporalGateInput gate_input;
    gate_input.valid = raw_valid;
    gate_input.state = static_cast<int>(result.state);
    gate_input.pose = result.t_registration_base;
    gate_input.confidence = result.confidence;
    gate_input.status = raw_status;
    const TemporalGateOutput gate_output = temporal_gate_.update(gate_input);
    result.valid = gate_output.valid;
    result.status = gate_output.status;
    result.consecutive_frames = gate_output.consecutive_frames;
    if (!result.valid) {
        result.state = BaseState::kUnknown;
    }
    return result;
}

PointCloud::ConstPtr
BaseLocalizer::cropSceneToSearchRegion(const PointCloud& scene_registration,
                                       const Eigen::Isometry3d& t_registration_base_initial) const {
    const PointCloud::Ptr finite_scene = finiteCopy(scene_registration);
    if (finite_scene->empty() || !configIsValid() ||
        !isFiniteTransform(t_registration_base_initial) || !template_bounds_valid_ ||
        !open_evaluator_.hasTemplate() || !closed_evaluator_.hasTemplate()) {
        return finite_scene;
    }

    Eigen::Vector3d minimum_registration =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d maximum_registration =
        Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    for (int x_index = 0; x_index < 2; ++x_index) {
        for (int y_index = 0; y_index < 2; ++y_index) {
            for (int z_index = 0; z_index < 2; ++z_index) {
                const Eigen::Vector3d point_base(
                    x_index == 0 ? template_minimum_base_.x() : template_maximum_base_.x(),
                    y_index == 0 ? template_minimum_base_.y() : template_maximum_base_.y(),
                    z_index == 0 ? template_minimum_base_.z() : template_maximum_base_.z());
                const Eigen::Vector3d point_registration = t_registration_base_initial * point_base;
                minimum_registration = minimum_registration.cwiseMin(point_registration);
                maximum_registration = maximum_registration.cwiseMax(point_registration);
            }
        }
    }

    if (!minimum_registration.allFinite() || !maximum_registration.allFinite()) {
        return finite_scene;
    }
    const double bounded_rotation = std::min(config_.max_rotation_delta_rad, std::acos(-1.0));
    const double rotation_displacement =
        2.0 * template_maximum_radius_base_ * std::sin(0.5 * bounded_rotation);
    const double padding = config_.max_translation_delta_m +
                           config_.icp_max_correspondence_distance_m + rotation_displacement;
    minimum_registration.array() -= padding;
    maximum_registration.array() += padding;

    PointCloud::Ptr cropped(new PointCloud);
    cropped->header = finite_scene->header;
    cropped->sensor_origin_ = finite_scene->sensor_origin_;
    cropped->sensor_orientation_ = finite_scene->sensor_orientation_;
    cropped->reserve(finite_scene->size());
    for (const PointT& point : *finite_scene) {
        const Eigen::Vector3d position(point.x, point.y, point.z);
        if ((position.array() >= minimum_registration.array()).all() &&
            (position.array() <= maximum_registration.array()).all()) {
            cropped->push_back(point);
        }
    }
    cropped->width = static_cast<std::uint32_t>(cropped->size());
    cropped->height = 1U;
    cropped->is_dense = true;
    return cropped;
}

bool BaseLocalizer::configIsValid() const noexcept {
    return config_.icp_max_iterations > 0 &&
           std::isfinite(config_.icp_max_correspondence_distance_m) &&
           config_.icp_max_correspondence_distance_m > 0.0 &&
           std::isfinite(config_.icp_transformation_epsilon) &&
           config_.icp_transformation_epsilon >= 0.0 &&
           std::isfinite(config_.icp_euclidean_fitness_epsilon) &&
           config_.icp_euclidean_fitness_epsilon >= 0.0 &&
           std::isfinite(config_.max_translation_delta_m) &&
           config_.max_translation_delta_m >= 0.0 &&
           std::isfinite(config_.max_rotation_delta_rad) && config_.max_rotation_delta_rad >= 0.0 &&
           std::isfinite(config_.min_state_score_margin) && config_.min_state_score_margin >= 0.0;
}

BaseCandidate BaseLocalizer::alignCandidate(const PointCloud::ConstPtr& scene_registration,
                                            const Eigen::Isometry3d& t_registration_base_initial,
                                            const BaseState state,
                                            const TemplateAlignmentEvaluator& evaluator) const {
    BaseCandidate candidate;
    candidate.state = state;
    candidate.t_registration_base = t_registration_base_initial;
    if (!configIsValid()) {
        candidate.status = LocalizationStatus::kInvalidConfiguration;
        return candidate;
    }
    if (!evaluator.hasTemplate()) {
        candidate.status = LocalizationStatus::kMissingTemplate;
        return candidate;
    }
    if (!isFiniteTransform(t_registration_base_initial)) {
        candidate.status = LocalizationStatus::kNonFiniteTransform;
        return candidate;
    }

    if (!scene_registration || scene_registration->size() < config_.metrics.min_scene_points) {
        candidate.status = LocalizationStatus::kInsufficientScenePoints;
        return candidate;
    }

    // PCL estimates source->target. Here source is scene in registration and target
    // is the base template, hence the optimizer directly estimates t_base_registration.
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(scene_registration);
    icp.setInputTarget(evaluator.templateCloud());
    icp.setMaximumIterations(config_.icp_max_iterations);
    icp.setMaxCorrespondenceDistance(config_.icp_max_correspondence_distance_m);
    icp.setTransformationEpsilon(config_.icp_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(config_.icp_euclidean_fitness_epsilon);
    pcl::registration::TransformationEstimation2D<PointT, PointT, float>::Ptr
        planar_transformation_estimation(
            new pcl::registration::TransformationEstimation2D<PointT, PointT, float>);
    icp.setTransformationEstimation(planar_transformation_estimation);

    PointCloud aligned_scene;
    const Eigen::Isometry3d t_base_registration_initial = t_registration_base_initial.inverse();
    icp.align(aligned_scene, t_base_registration_initial.matrix().cast<float>());
    candidate.optimizer_converged = icp.hasConverged();
    candidate.icp_fitness_score = icp.getFitnessScore(config_.icp_max_correspondence_distance_m);

    Eigen::Isometry3d t_base_registration = Eigen::Isometry3d::Identity();
    t_base_registration.matrix() = icp.getFinalTransformation().cast<double>();
    if (!isFiniteTransform(t_base_registration)) {
        candidate.status = LocalizationStatus::kNonFiniteTransform;
        return candidate;
    }
    candidate.t_registration_base = t_base_registration.inverse();

    const Eigen::Isometry3d initial_to_estimated =
        t_registration_base_initial.inverse() * candidate.t_registration_base;
    candidate.translation_delta_m = initial_to_estimated.translation().norm();
    candidate.rotation_delta_rad = rotationAngle(initial_to_estimated.linear());

    const AlignmentEvaluation evaluation =
        evaluator.evaluate(*scene_registration, candidate.t_registration_base, config_.metrics);
    candidate.metrics = evaluation.metrics;

    if (config_.require_icp_convergence && !candidate.optimizer_converged) {
        candidate.status = LocalizationStatus::kIcpNotConverged;
    } else if (candidate.translation_delta_m > config_.max_translation_delta_m) {
        candidate.status = LocalizationStatus::kTranslationBoundExceeded;
    } else if (candidate.rotation_delta_rad > config_.max_rotation_delta_rad) {
        candidate.status = LocalizationStatus::kRotationBoundExceeded;
    } else if (!evaluation.valid) {
        candidate.status = evaluation.status;
    } else {
        candidate.valid = true;
        candidate.status = LocalizationStatus::kValid;
    }
    return candidate;
}

} // namespace dart_vision::lidar
