#include "dart_lidar_localization/module/rail_module_localizer.hpp"

#include <gtest/gtest.h>

namespace dart_vision::lidar::localization {
namespace {

PointCloud::Ptr makeModuleModel() {
    PointCloud::Ptr cloud(new PointCloud);
    for (int x = 0; x < 6; ++x) {
        for (int y = 0; y < 5; ++y) {
            PointT point;
            point.x = static_cast<float>(x) * 0.008F;
            point.y = static_cast<float>(y) * 0.009F;
            point.z = static_cast<float>((x + 2 * y) % 4) * 0.007F;
            point.intensity = 1.0F;
            cloud->push_back(point);
        }
    }
    return cloud;
}

TEST(RailModuleLocalizer, RecoversPrismaticJointPosition) {
    constexpr double expected_position_m = 0.437;
    const PointCloud::Ptr model = makeModuleModel();
    PointCloud::Ptr observation(new PointCloud(*model));
    for (auto& point : *observation) {
        point.x += static_cast<float>(expected_position_m);
    }
    for (int index = 0; index < 20; ++index) {
        PointT clutter;
        clutter.x = 0.8F + 0.002F * static_cast<float>(index);
        clutter.y = 0.08F;
        clutter.z = 0.06F;
        observation->push_back(clutter);
    }

    ModuleLocalizationParameters parameters;
    parameters.coarse_step_m = 0.01;
    parameters.fine_step_m = 0.001;
    parameters.fine_half_window_m = 0.02;
    parameters.max_correspondence_distance_m = 0.012;
    parameters.max_rmse_m = 0.006;
    parameters.min_overlap_ratio = 0.9;
    parameters.min_correspondences = 25U;

    RailModuleLocalizer localizer(parameters, model);
    const auto result = localizer.locate(observation, Eigen::Isometry3d::Identity());
    ASSERT_TRUE(result.available) << result.message;
    EXPECT_NEAR(result.position_m, expected_position_m, 0.0011);
    EXPECT_GE(result.metrics.overlap_ratio, 0.9);
}

} // namespace
} // namespace dart_vision::lidar::localization
