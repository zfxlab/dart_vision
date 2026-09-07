#include "dart_lidar_localization/base/base_registrar.hpp"

#include "dart_lidar_localization/base/cloud_pyramid.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace dart_vision::lidar::localization {
namespace {

std::vector<double> ndtLeafSizes(const std::vector<NdtLevelParameters>& levels) {
    std::vector<double> result;
    result.reserve(levels.size());
    for (const auto& level : levels) {
        result.push_back(level.voxel_leaf_size_m);
    }
    return result;
}

std::vector<double> gicpLeafSizes(const std::vector<GicpLevelParameters>& levels) {
    std::vector<double> result;
    result.reserve(levels.size());
    for (const auto& level : levels) {
        result.push_back(level.voxel_leaf_size_m);
    }
    return result;
}

bool validNdtLevel(const NdtLevelParameters& level) {
    return std::isfinite(level.voxel_leaf_size_m) && level.voxel_leaf_size_m > 0.0 &&
           std::isfinite(level.resolution_m) && level.resolution_m > 0.0 &&
           std::isfinite(level.step_size_m) && level.step_size_m > 0.0 &&
           std::isfinite(level.transformation_epsilon) &&
           level.transformation_epsilon > 0.0 && level.max_iterations > 0;
}

bool validGicpLevel(const GicpLevelParameters& level) {
    return std::isfinite(level.voxel_leaf_size_m) && level.voxel_leaf_size_m > 0.0 &&
           std::isfinite(level.max_correspondence_distance_m) &&
           level.max_correspondence_distance_m > 0.0 &&
           std::isfinite(level.transformation_epsilon) &&
           level.transformation_epsilon > 0.0 && std::isfinite(level.fitness_epsilon) &&
           level.fitness_epsilon > 0.0 && level.max_iterations > 0;
}

Eigen::Vector3d rpyOf(const Eigen::Isometry3d& transform) {
    const Eigen::Vector3d ypr = transform.linear().eulerAngles(2, 1, 0);
    return Eigen::Vector3d(ypr.z(), ypr.y(), ypr.x());
}

} // namespace

BaseRegistrar::BaseRegistrar(BaseRegistrationParameters parameters,
                             PointCloud::ConstPtr model)
    : parameters_(std::move(parameters)), model_(std::move(model)) {
    if (!model_ || model_->empty()) {
        throw std::invalid_argument("Registration model must not be empty");
    }
    if (parameters_.gicp_levels.empty()) {
        throw std::invalid_argument("At least one GICP level is required");
    }
    if (parameters_.coarse_enabled && parameters_.ndt_levels.empty()) {
        throw std::invalid_argument("Coarse registration requires at least one NDT level");
    }
    for (const auto& level : parameters_.ndt_levels) {
        if (!validNdtLevel(level)) {
            throw std::invalid_argument("Invalid NDT level parameter");
        }
    }
    for (const auto& level : parameters_.gicp_levels) {
        if (!validGicpLevel(level)) {
            throw std::invalid_argument("Invalid GICP level parameter");
        }
    }
    if (!std::isfinite(parameters_.max_total_time_ms) ||
        parameters_.max_total_time_ms <= 0.0 || parameters_.min_input_points == 0U) {
        throw std::invalid_argument("Invalid registration runtime parameter");
    }

    if (parameters_.coarse_enabled) {
        coarse_models_ = CloudPyramid::build(model_, ndtLeafSizes(parameters_.ndt_levels));
    }
    fine_models_ = CloudPyramid::build(model_, gicpLeafSizes(parameters_.gicp_levels));
}

BaseRegistrationResult BaseRegistrar::align(
    const PointCloud::ConstPtr& observation,
    const Eigen::Isometry3d& initial_target_from_observation) const {
    const auto started_at = std::chrono::steady_clock::now();
    BaseRegistrationResult result;
    result.target_from_source = initial_target_from_observation;
    result.observation_from_model = initial_target_from_observation.inverse();

    const auto finish = [&](BaseRegistrationStatus status, std::string message) {
        result.status = status;
        result.message = std::move(message);
        result.elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started_at)
                                .count();
        return result;
    };

    if (!observation || observation->size() < parameters_.min_input_points) {
        return finish(BaseRegistrationStatus::kInvalidInput,
                      "observation does not contain enough points");
    }

    Eigen::Isometry3d current = initial_target_from_observation;
    if (parameters_.coarse_enabled) {
        result.coarse_used = true;
        const auto sources =
            CloudPyramid::build(observation, ndtLeafSizes(parameters_.ndt_levels));
        for (std::size_t index = 0U; index < parameters_.ndt_levels.size(); ++index) {
            const BaseStageResult stage = ndt_.align(
                sources[index], coarse_models_[index], current, parameters_.ndt_levels[index]);
            result.total_iterations += stage.iterations;
            if (!stage.converged || !stage.target_from_source.matrix().allFinite()) {
                return finish(BaseRegistrationStatus::kCoarseFailed,
                              "NDT level " + std::to_string(index) + " did not converge");
            }
            current = projectToAllowedDof(stage.target_from_source,
                                          initial_target_from_observation);
            if (timeBudgetExceeded(started_at)) {
                return finish(BaseRegistrationStatus::kTimeBudgetExceeded,
                              "time budget exceeded after NDT level " +
                                  std::to_string(index));
            }
        }
    }

    const auto fine_sources =
        CloudPyramid::build(observation, gicpLeafSizes(parameters_.gicp_levels));
    for (std::size_t index = 0U; index < parameters_.gicp_levels.size(); ++index) {
        const BaseStageResult stage = gicp_.align(
            fine_sources[index], fine_models_[index], current, parameters_.gicp_levels[index]);
        result.total_iterations += stage.iterations;
        if (!stage.converged || !stage.target_from_source.matrix().allFinite()) {
            return finish(BaseRegistrationStatus::kFineFailed,
                          "GICP level " + std::to_string(index) + " did not converge");
        }
        current =
            projectToAllowedDof(stage.target_from_source, initial_target_from_observation);
        if (timeBudgetExceeded(started_at)) {
            return finish(BaseRegistrationStatus::kTimeBudgetExceeded,
                          "time budget exceeded after GICP level " +
                              std::to_string(index));
        }
    }

    result.target_from_source = current;
    result.observation_from_model = current.inverse();
    result.metrics = validator_.evaluate(fine_sources.back(),
                                         fine_models_.back(),
                                         current,
                                         initial_target_from_observation,
                                         parameters_.validation);
    std::string rejection_reason;
    if (!validator_.accept(result.metrics, parameters_.validation, rejection_reason)) {
        return finish(BaseRegistrationStatus::kQualityRejected, rejection_reason);
    }
    return finish(BaseRegistrationStatus::kSuccess, baseStatusMessage(BaseRegistrationStatus::kSuccess));
}

Eigen::Isometry3d BaseRegistrar::projectToAllowedDof(
    const Eigen::Isometry3d& target_from_observation,
    const Eigen::Isometry3d& initial_target_from_observation) const {
    if (parameters_.dof_mode == DofMode::kSixDof) {
        return target_from_observation;
    }

    Eigen::Isometry3d observation_from_model = target_from_observation.inverse();
    const Eigen::Isometry3d initial_observation_from_model =
        initial_target_from_observation.inverse();
    const Eigen::Vector3d candidate_rpy = rpyOf(observation_from_model);
    const Eigen::Vector3d initial_rpy = rpyOf(initial_observation_from_model);
    observation_from_model.linear() =
        (Eigen::AngleAxisd(candidate_rpy.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(initial_rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(initial_rpy.x(), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    if (parameters_.dof_mode == DofMode::kXyYaw) {
        observation_from_model.translation().z() =
            initial_observation_from_model.translation().z();
    }
    return observation_from_model.inverse();
}

bool BaseRegistrar::timeBudgetExceeded(
    const std::chrono::steady_clock::time_point& started_at) const {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started_at)
               .count() > parameters_.max_total_time_ms;
}

} // namespace dart_vision::lidar::localization
