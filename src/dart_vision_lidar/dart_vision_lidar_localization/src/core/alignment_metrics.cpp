#include "dart_vision_lidar_localization/core/alignment_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <pcl/common/point_tests.h>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dart_vision::lidar {
namespace {

bool validMetricConfig(const AlignmentMetricConfig& config) {
    const double weight_sum =
        config.residual_score_weight + config.inlier_score_weight + config.coverage_score_weight;
    return std::isfinite(config.nearest_neighbor_truncation_m) &&
           config.nearest_neighbor_truncation_m > 0.0 && std::isfinite(config.inlier_distance_m) &&
           config.inlier_distance_m > 0.0 &&
           config.inlier_distance_m <= config.nearest_neighbor_truncation_m &&
           std::isfinite(config.max_truncated_rmse_m) && config.max_truncated_rmse_m >= 0.0 &&
           std::isfinite(config.min_inlier_ratio) && config.min_inlier_ratio >= 0.0 &&
           config.min_inlier_ratio <= 1.0 && std::isfinite(config.min_template_coverage_ratio) &&
           config.min_template_coverage_ratio >= 0.0 && config.min_template_coverage_ratio <= 1.0 &&
           std::isfinite(config.min_score) && config.min_score >= 0.0 && config.min_score <= 1.0 &&
           config.min_scene_points > 0U && std::isfinite(weight_sum) && weight_sum > 0.0 &&
           config.residual_score_weight >= 0.0 && config.inlier_score_weight >= 0.0 &&
           config.coverage_score_weight >= 0.0;
}

} // namespace

std::string_view toString(const LocalizationStatus status) noexcept {
    switch (status) {
        case LocalizationStatus::kValid:
            return "valid";
        case LocalizationStatus::kMissingTemplate:
            return "missing_template";
        case LocalizationStatus::kInsufficientScenePoints:
            return "insufficient_scene_points";
        case LocalizationStatus::kInvalidConfiguration:
            return "invalid_configuration";
        case LocalizationStatus::kNonFiniteTransform:
            return "non_finite_transform";
        case LocalizationStatus::kIcpNotConverged:
            return "icp_not_converged";
        case LocalizationStatus::kTranslationBoundExceeded:
            return "translation_bound_exceeded";
        case LocalizationStatus::kRotationBoundExceeded:
            return "rotation_bound_exceeded";
        case LocalizationStatus::kResidualTooHigh:
            return "residual_too_high";
        case LocalizationStatus::kInlierRatioTooLow:
            return "inlier_ratio_too_low";
        case LocalizationStatus::kCoverageTooLow:
            return "coverage_too_low";
        case LocalizationStatus::kScoreTooLow:
            return "score_too_low";
        case LocalizationStatus::kAmbiguousState:
            return "ambiguous_state";
        case LocalizationStatus::kTemporalUnconfirmed:
            return "temporal_unconfirmed";
        case LocalizationStatus::kTemporalDiscontinuity:
            return "temporal_discontinuity";
        case LocalizationStatus::kNoSearchCandidate:
            return "no_search_candidate";
    }
    return "unknown";
}

TemplateAlignmentEvaluator::TemplateAlignmentEvaluator() : template_cloud_(new PointCloud) {}

TemplateAlignmentEvaluator::TemplateAlignmentEvaluator(const PointCloud::ConstPtr& template_cloud)
    : TemplateAlignmentEvaluator() {
    setTemplate(template_cloud);
}

void TemplateAlignmentEvaluator::setTemplate(const PointCloud::ConstPtr& template_cloud) {
    PointCloud::Ptr finite_template(new PointCloud);
    if (template_cloud) {
        finite_template->reserve(template_cloud->size());
        for (const auto& point : *template_cloud) {
            if (pcl::isFinite(point)) {
                finite_template->push_back(point);
            }
        }
    }
    finite_template->width = static_cast<std::uint32_t>(finite_template->size());
    finite_template->height = 1U;
    finite_template->is_dense = true;
    template_cloud_ = finite_template;
    if (!template_cloud_->empty()) {
        nearest_neighbor_tree_.setInputCloud(template_cloud_);
    }
}

bool TemplateAlignmentEvaluator::hasTemplate() const noexcept {
    return template_cloud_ && !template_cloud_->empty();
}

const PointCloud::ConstPtr& TemplateAlignmentEvaluator::templateCloud() const noexcept {
    return template_cloud_;
}

AlignmentEvaluation
TemplateAlignmentEvaluator::evaluate(const PointCloud& scene_registration,
                                     const Eigen::Isometry3d& t_registration_template,
                                     const AlignmentMetricConfig& config) const {
    AlignmentEvaluation evaluation;
    if (!hasTemplate()) {
        evaluation.status = LocalizationStatus::kMissingTemplate;
        return evaluation;
    }
    evaluation.metrics.template_points = template_cloud_->size();
    if (!validMetricConfig(config)) {
        evaluation.status = LocalizationStatus::kInvalidConfiguration;
        return evaluation;
    }
    if (!isFiniteTransform(t_registration_template)) {
        evaluation.status = LocalizationStatus::kNonFiniteTransform;
        return evaluation;
    }

    const Eigen::Isometry3d t_template_registration = t_registration_template.inverse();
    const double truncation_squared =
        config.nearest_neighbor_truncation_m * config.nearest_neighbor_truncation_m;
    const double inlier_squared = config.inlier_distance_m * config.inlier_distance_m;
    double truncated_squared_error_sum = 0.0;
    std::unordered_set<int> covered_template_indices;
    covered_template_indices.reserve(std::min(scene_registration.size(), template_cloud_->size()));
    std::vector<int> nearest_indices(1);
    std::vector<float> nearest_squared_distances(1);

    for (const auto& scene_point : scene_registration) {
        if (!pcl::isFinite(scene_point)) {
            continue;
        }
        ++evaluation.metrics.scene_points;
        const Eigen::Vector3d point_template =
            t_template_registration * Eigen::Vector3d(scene_point.x, scene_point.y, scene_point.z);
        PointT query;
        query.x = static_cast<float>(point_template.x());
        query.y = static_cast<float>(point_template.y());
        query.z = static_cast<float>(point_template.z());
        query.intensity = scene_point.intensity;

        if (nearest_neighbor_tree_.nearestKSearch(
                query, 1, nearest_indices, nearest_squared_distances) <= 0) {
            truncated_squared_error_sum += truncation_squared;
            continue;
        }
        const double squared_distance = static_cast<double>(nearest_squared_distances.front());
        truncated_squared_error_sum += std::min(squared_distance, truncation_squared);
        if (squared_distance <= inlier_squared) {
            ++evaluation.metrics.inlier_points;
            covered_template_indices.insert(nearest_indices.front());
        }
    }

    if (evaluation.metrics.scene_points < config.min_scene_points) {
        evaluation.status = LocalizationStatus::kInsufficientScenePoints;
        return evaluation;
    }

    evaluation.metrics.covered_template_points = covered_template_indices.size();
    const double scene_count = static_cast<double>(evaluation.metrics.scene_points);
    evaluation.metrics.truncated_rmse_m = std::sqrt(truncated_squared_error_sum / scene_count);
    evaluation.metrics.inlier_ratio =
        static_cast<double>(evaluation.metrics.inlier_points) / scene_count;
    evaluation.metrics.template_coverage_ratio =
        static_cast<double>(evaluation.metrics.covered_template_points) /
        static_cast<double>(evaluation.metrics.template_points);

    const double residual_quality = std::clamp(
        1.0 - evaluation.metrics.truncated_rmse_m / config.nearest_neighbor_truncation_m, 0.0, 1.0);
    const double weight_sum =
        config.residual_score_weight + config.inlier_score_weight + config.coverage_score_weight;
    evaluation.metrics.score =
        (config.residual_score_weight * residual_quality +
         config.inlier_score_weight * evaluation.metrics.inlier_ratio +
         config.coverage_score_weight * evaluation.metrics.template_coverage_ratio) /
        weight_sum;

    if (evaluation.metrics.truncated_rmse_m > config.max_truncated_rmse_m) {
        evaluation.status = LocalizationStatus::kResidualTooHigh;
    } else if (evaluation.metrics.inlier_ratio < config.min_inlier_ratio) {
        evaluation.status = LocalizationStatus::kInlierRatioTooLow;
    } else if (evaluation.metrics.template_coverage_ratio < config.min_template_coverage_ratio) {
        evaluation.status = LocalizationStatus::kCoverageTooLow;
    } else if (evaluation.metrics.score < config.min_score) {
        evaluation.status = LocalizationStatus::kScoreTooLow;
    } else {
        evaluation.valid = true;
        evaluation.status = LocalizationStatus::kValid;
    }
    return evaluation;
}

bool isFiniteTransform(const Eigen::Isometry3d& transform) noexcept {
    return transform.matrix().allFinite();
}

double rotationAngle(const Eigen::Matrix3d& rotation) noexcept {
    if (!rotation.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    const double cosine = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosine);
}

} // namespace dart_vision::lidar
