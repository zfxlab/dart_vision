#include "dart_camera/pixel_center_ema.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

using dart_vision::camera::PixelCenterEma;
using dart_vision::camera::PixelCenterEmaConfig;

TEST(PixelCenterEma, InitializesImmediatelyAndUsesElapsedTimeWithIndependentAxes) {
    PixelCenterEma filter({true, 0.5, 2.0, 2.0});
    constexpr std::int64_t epoch = 1794744624000000000LL;
    EXPECT_EQ(filter.update({380, 520}, epoch), cv::Point2f(380, 520));
    filter.update({390, 500}, epoch + 10000000);
    filter.update({390, 500}, epoch + 170000000);
    const auto result = filter.update({390, 500}, epoch + 1000000000);
    EXPECT_NEAR(result.x, 390 - 10 * std::exp(-2.0), 3e-5);
    EXPECT_NEAR(result.y, 500 + 20 * std::exp(-0.5), 3e-5);
}

TEST(PixelCenterEma, ResetsOnDuplicateReversedAndLongGapTimestamps) {
    PixelCenterEma filter;
    filter.update({0, 0}, 0);
    EXPECT_EQ(filter.update({10, 20}, 0), cv::Point2f(10, 20));
    filter.update({0, 0}, 100000000);
    EXPECT_EQ(filter.update({30, 40}, 50000000), cv::Point2f(30, 40));
    EXPECT_EQ(filter.update({50, 60}, 3000000000), cv::Point2f(50, 60));
    filter.reset();
    EXPECT_EQ(filter.update({70, 80}, 3100000000), cv::Point2f(70, 80));
}

TEST(PixelCenterEma, ConfigurationChangesResetButIdenticalConfigurationKeepsHistory) {
    PixelCenterEma filter;
    filter.update({0, 0}, 0);
    filter.configure(filter.config());
    EXPECT_LT(filter.update({10, 20}, 100000000).x, 2);
    filter.configure({false, 1, 1, 2});
    EXPECT_EQ(filter.update({30, 40}, 200000000), cv::Point2f(30, 40));
    filter.configure({true, 1, 1, 2});
    EXPECT_EQ(filter.update({50, 60}, 300000000), cv::Point2f(50, 60));
    filter.configure({true, 2, 1, 2});
    EXPECT_EQ(filter.update({70, 80}, 400000000), cv::Point2f(70, 80));
}

TEST(PixelCenterEma, RejectsInvalidConfigurationWithoutChangingState) {
    PixelCenterEma filter;
    filter.update({0, 0}, 0);
    for (const double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_THROW(filter.configure({true, invalid, 1, 2}), std::invalid_argument);
        EXPECT_THROW(filter.configure({true, 1, invalid, 2}), std::invalid_argument);
        EXPECT_THROW(filter.configure({false, 1, 1, invalid}), std::invalid_argument);
    }
    EXPECT_NEAR(filter.update({10, 0}, 100000000).x, 10 * (1 - std::exp(-0.1)), 1e-6);
}

TEST(PixelCenterEma, InvalidObservationCannotPoisonNextOutput) {
    PixelCenterEma filter;
    filter.update({0, 0}, 0);
    EXPECT_THROW(filter.update({std::numeric_limits<float>::quiet_NaN(), 0}, 100),
                 std::invalid_argument);
    EXPECT_EQ(filter.update({10, 20}, 200), cv::Point2f(10, 20));
    EXPECT_THROW(filter.update({0, 0}, -1), std::invalid_argument);
    EXPECT_EQ(filter.update({30, 40}, 300), cv::Point2f(30, 40));
}
