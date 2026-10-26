#include "dart_stereo/stereo_triangulator.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace dart_vision::stereo;

TEST(Stereo3D, RecoversPointWithHorizontalVerticalAndObliqueBaselines) {
    const StereoTriangulator solver({});
    const cv::Vec3d target{10, 2, 3};
    for (const cv::Vec3d& baseline :
         {cv::Vec3d{0, 0.2, 0}, cv::Vec3d{0, 0, 0.2}, cv::Vec3d{0.1, 0.2, 0.3}}) {
        const cv::Vec3d left = -0.5 * baseline, right = 0.5 * baseline;
        const auto result = solver.triangulate(left, target - left, right, target - right);
        ASSERT_TRUE(result);
        EXPECT_LT(cv::norm(result->position_m - target), 1e-8);
        EXPECT_NEAR(result->ray_gap_m, 0, 1e-8);
        EXPECT_NEAR(result->distance_m, std::hypot(10, 2), 1e-8);
    }
}

TEST(Stereo3D, SkewMidpointIsEquivariantUnderRigidTransforms) {
    const StereoTriangulator solver({});
    const cv::Vec3d left{0, 0, 0}, right{1, -1, 0.08};
    const cv::Vec3d dl{1, 0, 0}, dr{0, 1, 0};
    const auto result = solver.triangulate(left, dl, right, dr);
    ASSERT_TRUE(result);
    EXPECT_LT(cv::norm(result->position_m - cv::Vec3d(1, 0, 0.04)), 1e-12);
    EXPECT_NEAR(result->ray_gap_m, 0.08, 1e-12);
    // Rotates the result behind reference x=0; still in front of both cameras.
    const cv::Matx33d rotation(0, 0, -1, 1, 0, 0, 0, -1, 0);
    const cv::Vec3d translation{-10, 2, 4};
    const auto transformed = solver.triangulate(rotation * left + translation, rotation * dl,
                                                rotation * right + translation, rotation * dr);
    ASSERT_TRUE(transformed);
    EXPECT_LT(cv::norm(transformed->position_m - (rotation * result->position_m + translation)),
              1e-10);
    EXPECT_NEAR(transformed->ray_gap_m, result->ray_gap_m, 1e-10);
    EXPECT_NEAR(transformed->left_ray_distance_m, result->left_ray_distance_m, 1e-10);
}

TEST(Stereo3D, RejectsDegenerateAndInvalidGeometry) {
    const StereoTriangulator solver({});
    StereoTriangulationDiagnostics diag;
    const auto reject = [&](cv::Vec3d l, cv::Vec3d dl, cv::Vec3d r, cv::Vec3d dr,
                            StereoTriangulationRejection reason) {
        EXPECT_FALSE(solver.triangulate(l, dl, r, dr, &diag));
        EXPECT_EQ(diag.rejection, reason);
    };
    reject({0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {0, 1, 0},
           StereoTriangulationRejection::degenerate_baseline);
    reject({0, 0, 0}, {0, 0, 0}, {0, 1, 0}, {1, 0, 0},
           StereoTriangulationRejection::invalid_bearing);
    for (double sign : {-1.0, 1.0})
        reject({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {sign, 0, 0},
               StereoTriangulationRejection::near_parallel_rays);
    reject({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, -1e-5, 0},
           StereoTriangulationRejection::ray_angle_too_small);
    reject({0, 0, 0}, {-1, 0, 0}, {1, -1, 0}, {0, 1, 0},
           StereoTriangulationRejection::behind_camera);
    reject({0, 0, 0}, {1, 0, 0}, {0.05, -1, 0}, {0, 1, 0},
           StereoTriangulationRejection::depth_below_minimum);
    reject({0, 0, 0}, {1, 0, 0}, {1, -1, 0.2}, {0, 1, 0},
           StereoTriangulationRejection::ray_gap_too_large);
    reject({0, 0, 0}, {1, 0, 0}, {51, -1, 0}, {0, 1, 0},
           StereoTriangulationRejection::camera_range_too_large);
    reject({0, 0, 0}, {std::numeric_limits<double>::quiet_NaN(), 0, 0}, {0, 1, 0}, {1, 0, 0},
           StereoTriangulationRejection::non_finite_input);
}

TEST(Stereo3D, RangeLimitUsesSpaceNotHorizontalProjection) {
    const StereoTriangulator solver({});
    const cv::Vec3d left{0, -0.1, 0}, right{0, 0.1, 0}, target{1, 0, 51};
    StereoTriangulationDiagnostics diag;
    EXPECT_FALSE(solver.triangulate(left, target - left, right, target - right, &diag));
    EXPECT_EQ(diag.rejection, StereoTriangulationRejection::camera_range_too_large);
}
