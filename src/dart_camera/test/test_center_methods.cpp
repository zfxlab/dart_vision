#include "dart_camera/green_light_detector.hpp"

#include <array>
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

using dart_vision::camera::GreenLightDetector;
using dart_vision::camera::GreenLightDetectorConfig;

namespace {
const std::array<std::string, 5> kMethods{"brightness_weighted", "contour_centroid",
                                          "enclosing_circle", "circle_fit", "ellipse_fit"};

cv::Mat gradientDisk(const cv::Point offset = {}) {
    cv::Mat image = cv::Mat::zeros(120, 120, CV_8UC3);
    for (int y = 45; y <= 75; ++y) {
        for (int x = 45; x <= 75; ++x) {
            if ((x - 60) * (x - 60) + (y - 60) * (y - 60) <= 15 * 15)
                image.at<cv::Vec3b>(y + offset.y, x + offset.x)[1] = 140 + 2 * (x - 60);
        }
    }
    return image;
}
} // namespace

TEST(CenterMethods, DefaultPreservesBackgroundSubtractedBrightnessCentroid) {
    auto image = gradientDisk();
    for (int y = 0; y < image.rows; ++y)
        for (int x = 0; x < image.cols; ++x)
            if (image.at<cv::Vec3b>(y, x)[1] == 0)
                image.at<cv::Vec3b>(y, x) = {20, 20, 20};
    const auto result = GreenLightDetector(GreenLightDetectorConfig{}).detect(image);
    ASSERT_TRUE(result.target);
    cv::Mat green;
    cv::extractChannel(image, green, 1);
    cv::Mat weights;
    green.convertTo(weights, CV_32F);
    weights -= 20.0F;
    const auto moments = cv::moments(weights);
    EXPECT_NEAR(result.target->center_px.x, moments.m10 / moments.m00, 1e-5);
    EXPECT_NEAR(result.target->center_px.y, moments.m01 / moments.m00, 1e-5);
    EXPECT_GT(result.target->center_px.x, 60.5);
    EXPECT_FALSE(result.target->center_fallback);
}

TEST(CenterMethods, GeometricCentersIgnoreInteriorBrightnessAndPreserveSelectionMetrics) {
    const auto image = gradientDisk();
    const auto baseline = GreenLightDetector(GreenLightDetectorConfig{}).detect(image);
    ASSERT_TRUE(baseline.target);
    for (const auto& method : kMethods) {
        SCOPED_TRACE(method);
        GreenLightDetectorConfig config;
        config.center_method = method;
        const auto result = GreenLightDetector(config).detect(image);
        ASSERT_TRUE(result.target);
        EXPECT_EQ(result.contours_count, baseline.contours_count);
        EXPECT_EQ(result.candidates.size(), baseline.candidates.size());
        EXPECT_DOUBLE_EQ(result.target->fit_score, baseline.target->fit_score);
        EXPECT_DOUBLE_EQ(result.target->radius_px, baseline.target->radius_px);
        EXPECT_DOUBLE_EQ(result.target->contrast_ratio, baseline.target->contrast_ratio);
        EXPECT_FALSE(result.target->center_fallback);
        if (method != "brightness_weighted") {
            EXPECT_NEAR(result.target->center_px.x, 60.0, 1e-4);
            EXPECT_NEAR(result.target->center_px.y, 60.0, 1e-4);
        }
    }
}

TEST(CenterMethods, EveryMethodReturnsOriginalImageCoordinatesAfterTranslation) {
    for (const auto& method : kMethods) {
        SCOPED_TRACE(method);
        GreenLightDetectorConfig config;
        config.center_method = method;
        const GreenLightDetector detector(config);
        const auto original = detector.detect(gradientDisk());
        const auto shifted = detector.detect(gradientDisk({7, -9}));
        ASSERT_TRUE(original.target);
        ASSERT_TRUE(shifted.target);
        EXPECT_NEAR(shifted.target->center_px.x - original.target->center_px.x, 7.0, 1e-4);
        EXPECT_NEAR(shifted.target->center_px.y - original.target->center_px.y, -9.0, 1e-4);
    }
}

TEST(CenterMethods, EllipseFitRecoversRotatedSubpixelCenter) {
    cv::Mat image = cv::Mat::zeros(120, 120, CV_8UC3);
    cv::ellipse(image, {964, 876}, {288, 192}, 30.0, 0.0, 360.0, {0, 230, 0}, cv::FILLED,
                cv::LINE_8, 4);
    GreenLightDetectorConfig config;
    config.center_method = "ellipse_fit";
    config.max_aspect_ratio = 2.0;
    config.min_circularity = 0.5;
    const auto result = GreenLightDetector(config).detect(image);
    ASSERT_TRUE(result.target);
    EXPECT_FALSE(result.target->center_fallback);
    EXPECT_NEAR(result.target->center_px.x, 60.25, 0.3);
    EXPECT_NEAR(result.target->center_px.y, 54.75, 0.3);
}

TEST(CenterMethods, DegenerateBrightnessWeightsExplicitlyFallBackToEnclosingCircle) {
    GreenLightDetectorConfig config;
    config.min_value = 0;
    config.min_green_excess = 0;
    config.min_inner_brightness = 0;
    config.min_contrast_ratio = 0;
    const cv::Mat image = cv::Mat::zeros(21, 21, CV_8UC3);
    const auto result = GreenLightDetector(config).detect(image);
    ASSERT_TRUE(result.target);
    EXPECT_TRUE(result.target->center_fallback);
    EXPECT_NEAR(result.target->center_px.x, 10.0, 1e-5);
    EXPECT_NEAR(result.target->center_px.y, 10.0, 1e-5);
}

TEST(CenterMethods, RejectsUnknownMethodInsteadOfSilentlyUsingDefault) {
    GreenLightDetectorConfig config;
    config.center_method = "typo";
    EXPECT_FALSE(config.isConfigValid());
    EXPECT_THROW({ GreenLightDetector detector(config); }, std::invalid_argument);
}
