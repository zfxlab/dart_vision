#include "dart_lidar_localization/module/rail_module_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dart_vision::lidar::localization {
namespace {

struct Candidate {
    double position_m{0.0};
    double objective{std::numeric_limits<double>::infinity()};
    ModuleLocalizationMetrics metrics;
};

} // namespace

RailModuleLocalizer::RailModuleLocalizer(ModuleLocalizationParameters parameters,
                                         PointCloud::ConstPtr module_model)
    : parameters_(std::move(parameters)), module_model_(std::move(module_model)) {
    if (!module_model_ || module_model_->empty()) {
        throw std::invalid_argument("Moving-module model must not be empty");
    }
    if (!std::isfinite(parameters_.min_position_m) || !std::isfinite(parameters_.max_position_m) ||
        parameters_.min_position_m >= parameters_.max_position_m ||
        !std::isfinite(parameters_.coarse_step_m) || parameters_.coarse_step_m <= 0.0 ||
        !std::isfinite(parameters_.fine_step_m) || parameters_.fine_step_m <= 0.0 ||
        !std::isfinite(parameters_.fine_half_window_m) || parameters_.fine_half_window_m < 0.0 ||
        !std::isfinite(parameters_.max_correspondence_distance_m) ||
        parameters_.max_correspondence_distance_m <= 0.0 || parameters_.min_correspondences == 0U ||
        !std::isfinite(parameters_.max_rmse_m) || parameters_.max_rmse_m <= 0.0 ||
        !std::isfinite(parameters_.min_overlap_ratio) || parameters_.min_overlap_ratio < 0.0 ||
        parameters_.min_overlap_ratio > 1.0 || !std::isfinite(parameters_.ambiguity_separation_m) ||
        parameters_.ambiguity_separation_m <= 0.0 ||
        !std::isfinite(parameters_.min_objective_gap_m) || parameters_.min_objective_gap_m < 0.0 ||
        !parameters_.roi_padding_m.allFinite() || (parameters_.roi_padding_m.array() < 0.0).any()) {
        throw std::invalid_argument("Invalid moving-module localization parameter");
    }

    model_min_.setConstant(std::numeric_limits<double>::infinity());
    model_max_.setConstant(-std::numeric_limits<double>::infinity());
    for (const auto& point : *module_model_) {
        const Eigen::Vector3d value(point.x, point.y, point.z);
        model_min_ = model_min_.cwiseMin(value);
        model_max_ = model_max_.cwiseMax(value);
    }
}

ModuleLocalizationResult
RailModuleLocalizer::locate(const PointCloud::ConstPtr& observation_in_reference,
                            const Eigen::Isometry3d& reference_from_rail) const {
    ModuleLocalizationResult result;
    if (!observation_in_reference || observation_in_reference->empty() ||
        !reference_from_rail.matrix().allFinite()) {
        result.message = "module observation is empty";
        return result;
    }

    PointCloud::Ptr observation_in_rail(new PointCloud);
    pcl::transformPointCloud(*observation_in_reference,
                             *observation_in_rail,
                             reference_from_rail.inverse().matrix().cast<float>());

    PointCloud::Ptr roi(new PointCloud);
    const Eigen::Vector3d roi_min = model_min_ +
                                    Eigen::Vector3d(parameters_.min_position_m, 0.0, 0.0) -
                                    parameters_.roi_padding_m;
    const Eigen::Vector3d roi_max = model_max_ +
                                    Eigen::Vector3d(parameters_.max_position_m, 0.0, 0.0) +
                                    parameters_.roi_padding_m;
    roi->reserve(observation_in_rail->size());
    for (const auto& point : *observation_in_rail) {
        const Eigen::Vector3d value(point.x, point.y, point.z);
        if ((value.array() >= roi_min.array()).all() && (value.array() <= roi_max.array()).all()) {
            roi->push_back(point);
        }
    }
    roi->width = static_cast<std::uint32_t>(roi->size());
    roi->height = 1U;
    roi->is_dense = true;
    if (roi->size() < parameters_.min_correspondences) {
        result.message = "module rail ROI contains too few observed points";
        return result;
    }

    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(roi);
    const double maximum_squared =
        parameters_.max_correspondence_distance_m * parameters_.max_correspondence_distance_m;

    const auto score = [&](const double position_m) {
        Candidate candidate;
        candidate.position_m = position_m;
        double squared_error_sum = 0.0;
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        for (const auto& model_point : *module_model_) {
            PointT query = model_point;
            query.x += static_cast<float>(position_m);
            if (tree.nearestKSearch(query, 1, indices, squared_distances) == 1 &&
                static_cast<double>(squared_distances.front()) <= maximum_squared) {
                squared_error_sum += squared_distances.front();
                ++candidate.metrics.correspondence_count;
            }
        }
        candidate.metrics.overlap_ratio =
            static_cast<double>(candidate.metrics.correspondence_count) /
            static_cast<double>(module_model_->size());
        if (candidate.metrics.correspondence_count > 0U) {
            candidate.metrics.rmse_m = std::sqrt(
                squared_error_sum / static_cast<double>(candidate.metrics.correspondence_count));
            candidate.objective =
                candidate.metrics.rmse_m +
                parameters_.max_correspondence_distance_m * (1.0 - candidate.metrics.overlap_ratio);
        } else {
            candidate.metrics.rmse_m = std::numeric_limits<double>::infinity();
        }
        return candidate;
    };

    Candidate best;
    std::vector<Candidate> coarse_candidates;
    for (double position = parameters_.min_position_m;
         position <= parameters_.max_position_m + 0.5 * parameters_.coarse_step_m;
         position += parameters_.coarse_step_m) {
        const Candidate candidate = score(std::min(position, parameters_.max_position_m));
        coarse_candidates.push_back(candidate);
        if (candidate.objective < best.objective) {
            best = candidate;
        }
    }
    if (!std::isfinite(best.objective)) {
        result.message = "module coarse search found no correspondences";
        return result;
    }

    const double fine_min =
        std::max(parameters_.min_position_m, best.position_m - parameters_.fine_half_window_m);
    const double fine_max =
        std::min(parameters_.max_position_m, best.position_m + parameters_.fine_half_window_m);
    for (double position = fine_min; position <= fine_max + 0.5 * parameters_.fine_step_m;
         position += parameters_.fine_step_m) {
        const Candidate candidate = score(std::min(position, fine_max));
        if (candidate.objective < best.objective) {
            best = candidate;
        }
    }

    result.position_m = best.position_m;
    result.has_candidate = true;
    result.metrics = best.metrics;
    result.metrics.search_boundary_hit =
        std::abs(result.position_m - parameters_.min_position_m) <= parameters_.fine_step_m ||
        std::abs(result.position_m - parameters_.max_position_m) <= parameters_.fine_step_m;
    std::string reason;
    result.available = validator_.accept(result, parameters_, reason);
    result.message = result.available ? "success" : reason;
    // 对与最优位置分离的第二候选做细搜，避免两处相似目标/运动拖影被强行选中。
    if (result.available && parameters_.min_objective_gap_m > 0.0) {
        Candidate alternative;
        for (const auto& candidate : coarse_candidates) {
            if (std::abs(candidate.position_m - best.position_m) >=
                    parameters_.ambiguity_separation_m &&
                candidate.objective < alternative.objective) {
                alternative = candidate;
            }
        }
        if (std::isfinite(alternative.objective)) {
            const double lower = std::max(parameters_.min_position_m,
                                          alternative.position_m - parameters_.fine_half_window_m);
            const double upper = std::min(parameters_.max_position_m,
                                          alternative.position_m + parameters_.fine_half_window_m);
            for (double position = lower; position <= upper; position += parameters_.fine_step_m) {
                if (std::abs(position - best.position_m) < parameters_.ambiguity_separation_m) {
                    continue;
                }
                const auto candidate = score(position);
                if (candidate.objective < alternative.objective) {
                    alternative = candidate;
                }
            }
            if (alternative.objective - best.objective < parameters_.min_objective_gap_m) {
                result.available = false;
                result.message =
                    "module position is ambiguous: separated candidates have similar scores";
            }
        }
    }
    return result;
}

} // namespace dart_vision::lidar::localization
