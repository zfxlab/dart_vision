#include "dart_aiming/aiming_core.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace dart_vision::aiming;

TEST(Aiming3D, PreservesSignedVerticalDifferenceThroughModelAndOffset) {
    const QuadraticAimModel model({0, 1, 0}, {0, 1, 0});
    for (double height : {-2.0, 0.0, 3.0}) {
        const auto geometric = solve(3, 4, height);
        ASSERT_TRUE(geometric);
        EXPECT_DOUBLE_EQ(geometric->horizontal_distance_m, 5);
        EXPECT_DOUBLE_EQ(geometric->vertical_difference_m, height);
        const auto modeled = model.apply(*geometric);
        ASSERT_TRUE(modeled);
        const auto final = applyDartOffset(*modeled, 0.1);
        ASSERT_TRUE(final);
        EXPECT_DOUBLE_EQ(final->horizontal_distance_m, 5);
        EXPECT_DOUBLE_EQ(final->vertical_difference_m, height);
        EXPECT_NEAR(final->yaw_error_rad, std::atan2(4, 3) + 0.1, 1e-12);
    }
}

TEST(Aiming3D, RejectsNonFiniteHeightAndTargetsBehindLauncher) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(solve(1, 0, nan));
    EXPECT_FALSE(solve(-1, 0, 2));
    EXPECT_FALSE(solve(0, 0, 2));
    EXPECT_FALSE(QuadraticAimModel({0, 1, 0}, {0, 1, 0}).apply({0, 1, nan}));
    EXPECT_FALSE(applyDartOffset({0, 1, nan}, 0));
}
