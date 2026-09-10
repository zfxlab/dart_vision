#include <gtest/gtest.h>

#include "dart_lidar_localization/calibration/base_validator.hpp"

namespace dart_vision::lidar::localization {
namespace {

TEST(BaseValidator, InitialDeviationIsOptional) {
    BaseRegistrationMetrics metrics;
    metrics.rmse_m = 0.005;
    metrics.overlap_ratio = 0.9;
    metrics.correspondence_count = 1000U;
    metrics.translation_from_initial_m = 0.5;
    metrics.yaw_from_initial_rad = 0.2;

    BaseValidationParameters parameters;
    parameters.max_rmse_m = 0.02;
    parameters.min_overlap_ratio = 0.5;
    parameters.min_correspondences = 100U;
    parameters.max_translation_from_initial_m = 0.08;
    parameters.max_yaw_from_initial_rad = 0.035;

    BaseValidator validator;
    std::string reason;
    EXPECT_TRUE(validator.accept(metrics, parameters, reason));

    parameters.enforce_initial_deviation_limits = true;
    EXPECT_FALSE(validator.accept(metrics, parameters, reason));
    EXPECT_FALSE(reason.empty());
}

TEST(BaseValidator, RejectsPoorPointCloudQuality) {
    BaseRegistrationMetrics metrics;
    metrics.rmse_m = 0.04;
    metrics.overlap_ratio = 0.9;
    metrics.correspondence_count = 1000U;
    BaseValidationParameters parameters;
    parameters.max_rmse_m = 0.02;
    parameters.min_overlap_ratio = 0.5;
    parameters.min_correspondences = 100U;

    BaseValidator validator;
    std::string reason;
    EXPECT_FALSE(validator.accept(metrics, parameters, reason));
    EXPECT_NE(reason.find("rmse"), std::string::npos);
}

} // namespace
} // namespace dart_vision::lidar::localization
