#include "dart_lidar_localization/calibration/base_validator.hpp"

#include <cmath>
#include <limits>
#include <pcl/kdtree/kdtree_flann.h>
#include <sstream>
#include <vector>

namespace dart_vision::lidar::localization {
namespace {

double yawOf(const Eigen::Isometry3d& transform) {
    const auto& rotation = transform.linear();
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

double normalizedAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace

BaseRegistrationMetrics BaseValidator::evaluate(const PointCloud::ConstPtr& source,
                                                const PointCloud::ConstPtr& target,
                                                const Eigen::Isometry3d& target_from_source,
                                                const Eigen::Isometry3d& initial_target_from_source,
                                                const BaseValidationParameters& parameters) const {
    BaseRegistrationMetrics metrics;
    if (!source || !target || source->empty() || target->empty()) {
        metrics.rmse_m = std::numeric_limits<double>::infinity();
        return metrics;
    }

    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(target);
    const double maximum_squared =
        parameters.correspondence_distance_m * parameters.correspondence_distance_m;
    double squared_error_sum = 0.0;
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);

    for (const auto& point : *source) {
        const Eigen::Vector3d transformed =
            target_from_source * Eigen::Vector3d(point.x, point.y, point.z);
        PointT query;
        query.x = static_cast<float>(transformed.x());
        query.y = static_cast<float>(transformed.y());
        query.z = static_cast<float>(transformed.z());
        if (tree.nearestKSearch(query, 1, indices, squared_distances) == 1 &&
            static_cast<double>(squared_distances[0]) <= maximum_squared) {
            squared_error_sum += static_cast<double>(squared_distances[0]);
            ++metrics.correspondence_count;
        }
    }

    metrics.overlap_ratio =
        static_cast<double>(metrics.correspondence_count) / static_cast<double>(source->size());
    metrics.rmse_m =
        metrics.correspondence_count == 0U
            ? std::numeric_limits<double>::infinity()
            : std::sqrt(squared_error_sum / static_cast<double>(metrics.correspondence_count));

    const Eigen::Isometry3d observation_from_model = target_from_source.inverse();
    const Eigen::Isometry3d initial_observation_from_model = initial_target_from_source.inverse();
    metrics.translation_from_initial_m =
        (observation_from_model.translation() - initial_observation_from_model.translation())
            .norm();
    metrics.yaw_from_initial_rad = std::abs(
        normalizedAngle(yawOf(observation_from_model) - yawOf(initial_observation_from_model)));
    metrics.search_boundary_hit =
        parameters.enforce_initial_deviation_limits &&
        (metrics.translation_from_initial_m >=
             parameters.boundary_ratio * parameters.max_translation_from_initial_m ||
         metrics.yaw_from_initial_rad >=
             parameters.boundary_ratio * parameters.max_yaw_from_initial_rad);
    return metrics;
}

bool BaseValidator::accept(const BaseRegistrationMetrics& metrics,
                           const BaseValidationParameters& parameters,
                           std::string& rejection_reason) const {
    std::ostringstream reason;
    if (!std::isfinite(metrics.rmse_m) || metrics.rmse_m > parameters.max_rmse_m) {
        reason << "rmse " << metrics.rmse_m << "m exceeds " << parameters.max_rmse_m << "m";
    } else if (metrics.overlap_ratio < parameters.min_overlap_ratio) {
        reason << "overlap " << metrics.overlap_ratio << " is below "
               << parameters.min_overlap_ratio;
    } else if (metrics.correspondence_count < parameters.min_correspondences) {
        reason << "correspondences " << metrics.correspondence_count << " is below "
               << parameters.min_correspondences;
    } else if (parameters.enforce_initial_deviation_limits &&
               metrics.translation_from_initial_m > parameters.max_translation_from_initial_m) {
        reason << "translation correction " << metrics.translation_from_initial_m << "m exceeds "
               << parameters.max_translation_from_initial_m << "m";
    } else if (parameters.enforce_initial_deviation_limits &&
               metrics.yaw_from_initial_rad > parameters.max_yaw_from_initial_rad) {
        reason << "yaw correction " << metrics.yaw_from_initial_rad << "rad exceeds "
               << parameters.max_yaw_from_initial_rad << "rad";
    } else if (parameters.enforce_initial_deviation_limits &&
               parameters.reject_if_search_boundary_hit && metrics.search_boundary_hit) {
        reason << "solution reached the configured search boundary";
    } else {
        rejection_reason.clear();
        return true;
    }
    rejection_reason = reason.str();
    return false;
}

} // namespace dart_vision::lidar::localization
