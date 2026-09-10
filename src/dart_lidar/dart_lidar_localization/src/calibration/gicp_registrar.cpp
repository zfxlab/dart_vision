#include "dart_lidar_localization/calibration/gicp_registrar.hpp"

#include <pcl/registration/gicp.h>

namespace dart_vision::lidar::localization {

BaseStageResult GicpRegistrar::align(const PointCloud::ConstPtr& source,
                                     const PointCloud::ConstPtr& target,
                                     const Eigen::Isometry3d& initial_guess,
                                     const GicpLevelParameters& parameters) const {
    BaseStageResult result;
    result.target_from_source = initial_guess;
    if (!source || !target || source->empty() || target->empty()) {
        return result;
    }

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target);
    gicp.setMaxCorrespondenceDistance(parameters.max_correspondence_distance_m);
    gicp.setTransformationEpsilon(parameters.transformation_epsilon);
    gicp.setEuclideanFitnessEpsilon(parameters.fitness_epsilon);
    gicp.setMaximumIterations(parameters.max_iterations);

    PointCloud aligned;
    gicp.align(aligned, initial_guess.matrix().cast<float>());
    result.converged = gicp.hasConverged();
    result.target_from_source = Eigen::Isometry3d(gicp.getFinalTransformation().cast<double>());
    // PCL 1.12的GICP没有NDT那样的getter，但在公有接口中导出了nr_iterations_。
    result.iterations = gicp.nr_iterations_;
    result.fitness_score = gicp.getFitnessScore(parameters.max_correspondence_distance_m);
    return result;
}

} // namespace dart_vision::lidar::localization
