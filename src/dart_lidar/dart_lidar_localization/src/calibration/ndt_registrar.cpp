#include "dart_lidar_localization/calibration/ndt_registrar.hpp"

#include <pcl/registration/ndt.h>

namespace dart_vision::lidar::localization {

BaseStageResult NdtRegistrar::align(const PointCloud::ConstPtr& source,
                                    const PointCloud::ConstPtr& target,
                                    const Eigen::Isometry3d& initial_guess,
                                    const NdtLevelParameters& parameters) const {
    BaseStageResult result;
    result.target_from_source = initial_guess;
    if (!source || !target || source->empty() || target->empty()) {
        return result;
    }

    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    ndt.setResolution(static_cast<float>(parameters.resolution_m));
    ndt.setStepSize(parameters.step_size_m);
    ndt.setTransformationEpsilon(parameters.transformation_epsilon);
    ndt.setMaximumIterations(parameters.max_iterations);

    PointCloud aligned;
    ndt.align(aligned, initial_guess.matrix().cast<float>());
    result.converged = ndt.hasConverged();
    result.target_from_source = Eigen::Isometry3d(ndt.getFinalTransformation().cast<double>());
    result.iterations = ndt.getFinalNumIteration();
    result.fitness_score = ndt.getFitnessScore();
    return result;
}

} // namespace dart_vision::lidar::localization
