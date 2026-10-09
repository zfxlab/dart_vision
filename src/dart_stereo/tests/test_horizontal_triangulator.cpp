#include "dart_stereo/stereo_triangulator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace dart_vision::stereo;

void check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

bool close(double a, double b) {
    return std::abs(a - b) < 1e-8;
}

int main() {
    StereoTriangulatorConfig config;
    config.max_height_gap_m = 0.11;
    StereoTriangulator solver(config);
    const cv::Vec3d left{0.025, 0.15, 0.015}, right{0.025, -0.15, 0.015};
    const cv::Vec3d target{10.0, 2.0, 3.0};
    auto result = solver.triangulate(left, target - left, right, target - right);
    if (!result) {
        std::cerr << "Exact spatial intersection must succeed\n";
        return EXIT_FAILURE;
    }
    check(cv::norm(result->position_m - target) < 1e-8, "Position must match known target");
    check(close(result->distance_m, std::hypot(10.0, 2.0)), "Distance must exclude height");

    const cv::Vec3d high_target{10.0, 2.0, 100.0};
    result = solver.triangulate(left, high_target - left, right, high_target - right);
    check(result.has_value() && close(result->distance_m, std::hypot(10.0, 2.0)),
          "Large height must not trigger horizontal distance limit");

    // 两路高度不同，水平交点必须保持不变，输出平均高度。
    const cv::Vec3d elevated{10.0, 2.0, 3.1};
    result = solver.triangulate(left, target - left, right, elevated - right);
    if (!result) {
        std::cerr << "Small height disagreement must succeed\n";
        return EXIT_FAILURE;
    }
    check(close(result->position_m[0], 10.0) && close(result->position_m[1], 2.0),
          "Vertical disagreement must not shift horizontal intersection");
    check(close(result->position_m[2], 3.05) && close(result->height_gap_m, 0.1),
          "Mean height and height gap must match ray heights");

    StereoTriangulationDiagnostics diagnostics;
    auto rejected = [&](const cv::Vec3d& l, const cv::Vec3d& r,
                        StereoTriangulationRejection reason) {
        check(!solver.triangulate(left, l, right, r, &diagnostics), "Invalid geometry accepted");
        check(diagnostics.rejection == reason, "Unexpected rejection reason");
    };
    rejected(target - left, cv::Vec3d{10.0, 2.0, 3.2} - right,
             StereoTriangulationRejection::height_gap_too_large);
    rejected({1, 0, 0}, {1, 0, 1}, StereoTriangulationRejection::ray_angle_too_small);
    rejected({1, 0, 0}, {-1, 0, 0}, StereoTriangulationRejection::near_parallel_rays);
    rejected({0, 0, 1}, target - right, StereoTriangulationRejection::invalid_bearing);
    rejected(left - target, right - target, StereoTriangulationRejection::behind_camera);
    rejected({std::numeric_limits<double>::quiet_NaN(), 0, 1}, target - right,
             StereoTriangulationRejection::non_finite_input);
    check(!solver.triangulate({0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, -1}, &diagnostics) &&
              diagnostics.rejection == StereoTriangulationRejection::degenerate_baseline,
          "Purely vertical baseline must be rejected");

    config.max_distance_m = 9.0;
    check(!StereoTriangulator(config).triangulate(left, target - left, right, target - right,
                                                  &diagnostics) &&
              diagnostics.rejection == StereoTriangulationRejection::distance_too_large,
          "Horizontal distance limit must apply");
    config.max_distance_m = 50.0;
    config.min_depth_m = 20.0;
    check(!StereoTriangulator(config).triangulate(left, target - left, right, target - right,
                                                  &diagnostics) &&
              diagnostics.rejection == StereoTriangulationRejection::depth_below_minimum,
          "Original spatial ray depth limit must apply");
    std::cout << "Horizontal triangulation checks passed\n";
}
