#include "dart_camera/green_light_detector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <utility>

namespace dart_vision::camera {
namespace {
constexpr float kOuterRadiusScale = 1.5f;

constexpr double kMinimumOuterBrightness = 1.0;

void fillEnclosedHoles(cv::Mat& mask) {
    cv::Mat padded_mask;
    cv::copyMakeBorder(mask, padded_mask, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::floodFill(padded_mask, cv::Point(0, 0), cv::Scalar(255));

    const cv::Mat flooded_without_border = padded_mask(cv::Rect(1, 1, mask.cols, mask.rows));
    cv::Mat holes;
    cv::bitwise_not(flooded_without_border, holes);
    cv::bitwise_or(mask, holes, mask);
}

double calculateMeanRadialError(const std::vector<cv::Point>& contour, const cv::Point2f& center_px,
                                const double radius_px) {
    if (contour.empty() || radius_px <= 0.0) {
        return 0.0;
    }

    double total_error_px = 0.0;

    for (const cv::Point& point : contour) {
        const cv::Point2f point_px{static_cast<float>(point.x), static_cast<float>(point.y)};
        const double distance_px = cv::norm(point_px - center_px);
        total_error_px += std::abs(distance_px - radius_px);
    }

    return total_error_px / static_cast<double>(contour.size());
}

std::optional<cv::Point2f>
calculateBrightnessWeightedCenter(const std::vector<cv::Point>& contour,
                                  const cv::Mat& green_channel,
                                  const double local_background_brightness) {
    const cv::Rect bounds =
        cv::boundingRect(contour) & cv::Rect(0, 0, green_channel.cols, green_channel.rows);
    if (bounds.empty() || !std::isfinite(local_background_brightness)) {
        return std::nullopt;
    }

    cv::Mat contour_mask = cv::Mat::zeros(bounds.size(), CV_8UC1);
    cv::drawContours(contour_mask, std::vector<std::vector<cv::Point>>{contour}, -1,
                     cv::Scalar(255), cv::FILLED, cv::LINE_8, cv::noArray(), 0, -bounds.tl());

    cv::Mat weights;
    green_channel(bounds).convertTo(weights, CV_32F);
    weights -= static_cast<float>(local_background_brightness);
    cv::threshold(weights, weights, 0.0, 0.0, cv::THRESH_TOZERO);
    weights.setTo(0.0f, contour_mask == 0);

    const cv::Moments weighted_moments = cv::moments(weights, false);
    if (!std::isfinite(weighted_moments.m00) || weighted_moments.m00 <= 0.0) {
        return std::nullopt;
    }

    const double center_x =
        static_cast<double>(bounds.x) + weighted_moments.m10 / weighted_moments.m00;
    const double center_y =
        static_cast<double>(bounds.y) + weighted_moments.m01 / weighted_moments.m00;
    if (!std::isfinite(center_x) || !std::isfinite(center_y)) {
        return std::nullopt;
    }

    return cv::Point2f{static_cast<float>(center_x), static_cast<float>(center_y)};
}

// 筛选仍使用 CHAIN_APPROX_SIMPLE。仅在拟合中心时恢复逐像素边界，避免长直边
// 只剩端点造成采样权重不均，并保持默认算法的轮廓、得分和筛选行为不变。
std::vector<cv::Point> denseBoundary(const std::vector<cv::Point>& contour) {
    const cv::Rect bounds = cv::boundingRect(contour);
    cv::Mat mask = cv::Mat::zeros(bounds.size(), CV_8UC1);
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255),
                     cv::FILLED, cv::LINE_8, cv::noArray(), 0, -bounds.tl());
    std::vector<std::vector<cv::Point>> boundaries;
    cv::findContours(mask, boundaries, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, bounds.tl());
    return boundaries.size() == 1U ? std::move(boundaries.front()) : std::vector<cv::Point>{};
}

std::optional<cv::Point2f> fitCircleCenter(const std::vector<cv::Point>& points) {
    if (points.size() < 3U)
        return std::nullopt;
    cv::Point2d origin{};
    for (const auto& point : points)
        origin += cv::Point2d(point);
    origin *= 1.0 / static_cast<double>(points.size());
    double scale = 0.0;
    for (const auto& point : points)
        scale = std::max(scale, cv::norm(cv::Point2d(point) - origin));
    if (!std::isfinite(scale) || scale <= 0.0)
        return std::nullopt;

    // 局部归一化后求解 2*a*x + 2*b*y + c = x^2 + y^2，中心为 (a,b)。
    // 这是代数最小二乘圆拟合，不是最小外接圆，也不是抗离群点拟合。
    cv::Mat design(static_cast<int>(points.size()), 3, CV_64F);
    cv::Mat rhs(static_cast<int>(points.size()), 1, CV_64F);
    for (int i = 0; i < design.rows; ++i) {
        const auto point = (cv::Point2d(points[i]) - origin) * (1.0 / scale);
        design.at<double>(i, 0) = 2.0 * point.x;
        design.at<double>(i, 1) = 2.0 * point.y;
        design.at<double>(i, 2) = 1.0;
        rhs.at<double>(i) = point.dot(point);
    }
    const cv::SVD decomposition(design);
    if (decomposition.w.at<double>(2) <= decomposition.w.at<double>(0) * 1e-8)
        return std::nullopt;
    cv::Mat solution;
    decomposition.backSubst(rhs, solution);
    const double a = solution.at<double>(0), b = solution.at<double>(1);
    const double radius_squared = a * a + b * b + solution.at<double>(2);
    if (!std::isfinite(radius_squared) || radius_squared <= 0.0)
        return std::nullopt;
    return cv::Point2f{static_cast<float>(origin.x + scale * a),
                       static_cast<float>(origin.y + scale * b)};
}

std::optional<cv::Point2f> calculateObservationCenter(const std::string& method,
                                                      const std::vector<cv::Point>& contour,
                                                      const cv::Mat& green_channel,
                                                      double background,
                                                      const cv::Point2f& enclosing_center) {
    std::optional<cv::Point2f> center;
    try {
        if (method == "enclosing_circle") {
            center = enclosing_center;
        } else if (method == "brightness_weighted") {
            center = calculateBrightnessWeightedCenter(contour, green_channel, background);
        } else if (method == "contour_centroid") {
            const auto moments = cv::moments(contour);
            if (std::isfinite(moments.m00) && moments.m00 > 0.0)
                center = cv::Point2f{static_cast<float>(moments.m10 / moments.m00),
                                     static_cast<float>(moments.m01 / moments.m00)};
        } else {
            const auto boundary = denseBoundary(contour);
            if (method == "circle_fit") {
                center = fitCircleCenter(boundary);
            } else if (method == "ellipse_fit" && boundary.size() >= 5U) {
                const auto ellipse = cv::fitEllipse(boundary);
                if (std::isfinite(ellipse.size.width) && std::isfinite(ellipse.size.height) &&
                    ellipse.size.width > 0.0 && ellipse.size.height > 0.0)
                    center = ellipse.center;
            }
        }
    } catch (const cv::Exception&) {
        return std::nullopt;
    }
    const cv::Rect bounds = cv::boundingRect(contour);
    if (!center || !std::isfinite(center->x) || !std::isfinite(center->y) || center->x < bounds.x ||
        center->x > bounds.x + bounds.width - 1 || center->y < bounds.y ||
        center->y > bounds.y + bounds.height - 1)
        return std::nullopt;
    return center;
}

} // namespace

// clang-format off
bool MaskCleanupConfig::isConfigValid() const {
    if (close_kernel_size < 0 || close_iterations < 0 ||
        (close_kernel_size > 0 && close_kernel_size % 2 == 0)) {
        return false;
    }
    if (open_kernel_size < 0 || open_iterations < 0 ||
        (open_kernel_size > 0 && open_kernel_size % 2 == 0)) {
        return false;
    }
    return true;
}

bool GreenLightDetectorConfig::isConfigValid() const {
    if (center_method != "brightness_weighted" && center_method != "contour_centroid" &&
        center_method != "enclosing_circle" && center_method != "circle_fit" &&
        center_method != "ellipse_fit") {
        return false;
    }
    if (min_hue < 0.0 || min_hue > 179.0 ||
        max_hue < 0.0 || max_hue > 179.0 ||
        min_hue > max_hue) {
        return false;
    }

    if (min_saturation < 0.0 || min_saturation > 255.0 ||
        min_value < 0.0 || min_value > 255.0) {
        return false;
    }

    if (min_green_excess < 0.0) {
        return false;
    }

    if (!std::isfinite(min_radius_px) ||
        !std::isfinite(max_radius_px) ||
        min_radius_px <= 0.0 ||
        max_radius_px < min_radius_px) {
        return false;
    }

    if (!std::isfinite(min_circularity) ||
        min_circularity < 0.0 ||
        min_circularity > 1.0) {
        return false;
    }

    if (!std::isfinite(max_aspect_ratio) || max_aspect_ratio < 1.0) {
        return false;
    }

    if (!std::isfinite(min_fill_ratio) || min_fill_ratio < 0.0 || min_fill_ratio > 1.0) {
        return false;
    }

    if (!std::isfinite(min_inner_brightness) ||
        min_inner_brightness < 0.0 ||
        min_inner_brightness > 255.0) {
        return false;
    }

    if (!std::isfinite(min_contrast_ratio) ||
        min_contrast_ratio < 0.0) {
        return false;
    }

    if (!cleanup.isConfigValid()) {
        return false;
    }

    return true;
}
// clang-format on

GreenLightDetector::GreenLightDetector(const GreenLightDetectorConfig& config) : config_(config) {
    if (!config_.isConfigValid()) {
        throw std::invalid_argument("Invalid light green_light_detector configuration.");
    }
}

cv::Mat GreenLightDetector::normalizeInput(const cv::Mat& image) const {
    if (image.type() != CV_8UC3 && image.type() != CV_8UC4) {
        throw std::invalid_argument(
            "Input image must be 3 or 4 channels image, either CV_8UC3 or CV_8UC4.");
    }

    if (image.type() == CV_8UC4) {
        cv::Mat bgr_img;
        cv::cvtColor(image, bgr_img, cv::COLOR_BGRA2BGR);
        return bgr_img;
    }

    return image;
}

GreenLightDetector::SegmentationResult
GreenLightDetector::makeGreenMask(const cv::Mat& bgr_img) const {
    if (bgr_img.empty() || bgr_img.type() != CV_8UC3) {
        throw std::invalid_argument("BGR image must be a non-empty CV_8UC3 image.");
    }

    std::vector<cv::Mat> channels;
    cv::split(bgr_img, channels);
    const cv::Mat& blue_channel = channels[0];
    const cv::Mat& green_channel = channels[1];
    const cv::Mat& red_channel = channels[2];

    // HSV 路径适合颜色和饱和度稳定、曝光正常的绿色区域。
    cv::Mat hsv_img;
    cv::cvtColor(bgr_img, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat hsv_green_mask;
    cv::inRange(hsv_img, cv::Scalar(config_.min_hue, config_.min_saturation, config_.min_value),
                cv::Scalar(config_.max_hue, 255, 255), hsv_green_mask);

    // 绿色优势路径保留低饱和或局部过曝但 G 仍显著高于 R、B 的发光区域；
    // 亮度门限用于排除暗部色彩噪声。
    cv::Mat green_excess;
    cv::addWeighted(green_channel, 2.0, red_channel, -1.0, 0.0, green_excess, CV_16S);
    cv::addWeighted(green_excess, 1.0, blue_channel, -1.0, 0.0, green_excess, CV_16S);

    cv::Mat green_excess_mask;
    cv::compare(green_excess, cv::Scalar(config_.min_green_excess), green_excess_mask, cv::CMP_GE);

    cv::Mat bright_green_mask;
    cv::compare(green_channel, cv::Scalar(config_.min_value), bright_green_mask, cv::CMP_GE);

    cv::bitwise_and(green_excess_mask, bright_green_mask, green_excess_mask);

    // 两条路径互补，取并集以兼顾正常曝光和高亮光源。
    cv::Mat binary_mask;
    cv::bitwise_or(hsv_green_mask, green_excess_mask, binary_mask);

    return SegmentationResult{green_channel, binary_mask};
}

void GreenLightDetector::cleanMask(cv::Mat& mask) const {
    if (mask.empty() || mask.type() != CV_8UC1) {
        throw std::invalid_argument("Mask must be a non-empty CV_8UC1 image.");
    }

    // 闭运算： 先膨胀，再腐蚀
    // 用于连接相邻白色区域、填补小裂缝
    if (config_.cleanup.close_kernel_size > 0 && config_.cleanup.close_iterations > 0) {
        const cv::Mat close_kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(config_.cleanup.close_kernel_size, config_.cleanup.close_kernel_size));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel, cv::Point(-1, -1),
                         config_.cleanup.close_iterations);
    }

    // 开运算： 先腐蚀，再膨胀
    // 用于去除小白色噪点和细小连接
    if (config_.cleanup.open_kernel_size > 0 && config_.cleanup.open_iterations > 0) {
        const cv::Mat open_kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(config_.cleanup.open_kernel_size, config_.cleanup.open_kernel_size));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel, cv::Point(-1, -1),
                         config_.cleanup.open_iterations);
    }

    if (config_.cleanup.fill_holes) {
        fillEnclosedHoles(mask);
    }
}

GreenLightDetector::CandidateExtractionResult
GreenLightDetector::extractCandidates(const cv::Mat& binary_mask,
                                      const cv::Mat& green_channel) const {
    if (binary_mask.empty() || binary_mask.type() != CV_8UC1) {
        throw std::invalid_argument("Binary mask must be a non-empty CV_8UC1 image.");
    }

    if (green_channel.empty() || green_channel.type() != CV_8UC1) {
        throw std::invalid_argument("Green channel must be a non-empty CV_8UC1 image.");
    }

    if (binary_mask.size() != green_channel.size()) {
        throw std::invalid_argument("Binary mask and green channel must have the same size.");
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<GreenLightCandidate> candidates;
    candidates.reserve(contours.size());

    for (const std::vector<cv::Point>& contour : contours) {
        if (contour.size() < 3) {
            continue;
        }

        const double area_px2 = cv::contourArea(contour);
        if (!std::isfinite(area_px2) || area_px2 <= 0.0) {
            continue;
        }

        const double perimeter_px = cv::arcLength(contour, true);
        if (!std::isfinite(perimeter_px) || perimeter_px <= 0.0) {
            continue;
        }

        const double circularity = 4.0 * CV_PI * area_px2 / (perimeter_px * perimeter_px);
        if (!std::isfinite(circularity) || circularity < config_.min_circularity) {
            continue;
        }

        cv::Point2f center_px;
        float radius_px_f{};
        cv::minEnclosingCircle(contour, center_px, radius_px_f);
        const double radius_px = static_cast<double>(radius_px_f);
        if (!std::isfinite(center_px.x) || !std::isfinite(center_px.y) ||
            !std::isfinite(radius_px) || radius_px < config_.min_radius_px ||
            radius_px > config_.max_radius_px) {
            continue;
        }

        const cv::Rect bounding_box = cv::boundingRect(contour);
        if (bounding_box.width <= 0 || bounding_box.height <= 0) {
            continue;
        }
        const double aspect_ratio =
            static_cast<double>(std::max(bounding_box.width, bounding_box.height)) /
            static_cast<double>(std::min(bounding_box.width, bounding_box.height));
        if (aspect_ratio > config_.max_aspect_ratio) {
            continue;
        }

        const double circle_area_px2 = CV_PI * radius_px * radius_px;
        if (!std::isfinite(circle_area_px2) || circle_area_px2 < 0.0) {
            continue;
        }

        const double fill_ratio = area_px2 / circle_area_px2;
        if (!std::isfinite(fill_ratio) || fill_ratio < config_.min_fill_ratio) {
            continue;
        }

        const cv::Point center_integer{cvRound(center_px.x), cvRound(center_px.y)};
        const int inner_radius = cvRound(radius_px);
        const int outer_radius = cvRound(radius_px * kOuterRadiusScale);
        // 包含完整外圆，并沿图像边界裁剪，保留原有圆环的像素选择和边缘统计语义。
        const cv::Rect roi =
            cv::Rect(center_integer.x - outer_radius, center_integer.y - outer_radius,
                     2 * outer_radius + 1, 2 * outer_radius + 1) &
            cv::Rect(0, 0, green_channel.cols, green_channel.rows);
        if (roi.empty()) {
            continue;
        }
        const cv::Point local_center = center_integer - roi.tl();
        const cv::Mat green_roi = green_channel(roi);
        cv::Mat inner_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
        cv::circle(inner_mask, local_center, inner_radius, cv::Scalar(255), cv::FILLED);
        cv::Mat outer_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
        cv::circle(outer_mask, local_center, outer_radius, cv::Scalar(255), cv::FILLED);
        cv::circle(outer_mask, local_center, inner_radius, cv::Scalar(0), cv::FILLED);

        if (cv::countNonZero(inner_mask) == 0 || cv::countNonZero(outer_mask) == 0) {
            continue;
        }

        const double mean_inner_brightness = cv::mean(green_roi, inner_mask)[0];
        const double mean_outer_brightness = cv::mean(green_roi, outer_mask)[0];
        // 用相邻的 1R～1.5R 圆环估计局部背景；分母钳制避免纯黑背景导致比值发散。
        const double contrast_ratio =
            mean_inner_brightness / std::max(mean_outer_brightness, kMinimumOuterBrightness);

        if (!std::isfinite(mean_inner_brightness) || !std::isfinite(mean_outer_brightness) ||
            !std::isfinite(contrast_ratio)) {
            continue;
        }
        if (mean_inner_brightness < config_.min_inner_brightness ||
            contrast_ratio < config_.min_contrast_ratio) {
            continue;
        }

        const auto estimated_center = calculateObservationCenter(
            config_.center_method, contour, green_channel, mean_outer_brightness, center_px);
        const cv::Point2f observation_center = estimated_center.value_or(center_px);

        const double mean_radial_error_px = calculateMeanRadialError(contour, center_px, radius_px);
        const double fit_score = std::clamp(1.0 - mean_radial_error_px / radius_px, 0.0, 1.0);

        candidates.push_back(GreenLightCandidate{
            observation_center, radius_px, area_px2, circularity, aspect_ratio, fill_ratio,
            mean_inner_brightness, mean_outer_brightness, contrast_ratio, mean_radial_error_px,
            fit_score, !estimated_center.has_value()});
    }

    return CandidateExtractionResult{std::move(candidates), contours.size()};
}

std::optional<GreenLightCandidate>
GreenLightDetector::selectBestCandidate(const std::vector<GreenLightCandidate>& candidates) const {
    if (candidates.empty()) {
        return std::nullopt;
    }

    const auto best_candidate =
        std::max_element(candidates.begin(), candidates.end(),
                         [](const GreenLightCandidate& left, const GreenLightCandidate& right) {
                             // 优先选择轮廓最贴合圆的候选，再以整体圆度和局部对比度消歧。
                             if (left.fit_score != right.fit_score) {
                                 return left.fit_score < right.fit_score;
                             }

                             if (left.circularity != right.circularity) {
                                 return left.circularity < right.circularity;
                             }

                             return left.contrast_ratio < right.contrast_ratio;
                         });

    return *best_candidate;
}

GreenLightDetectionResult GreenLightDetector::detect(const cv::Mat& image) const {
    const cv::Mat bgr_img = normalizeInput(image);

    SegmentationResult segmentation = makeGreenMask(bgr_img);

    cleanMask(segmentation.binary_mask);

    CandidateExtractionResult extraction =
        extractCandidates(segmentation.binary_mask, segmentation.green_channel);

    std::optional<GreenLightCandidate> target = selectBestCandidate(extraction.accepted_candidates);

    return GreenLightDetectionResult{target, std::move(extraction.accepted_candidates),
                                     extraction.contours_count,
                                     std::move(segmentation.binary_mask)};
}

} // namespace dart_vision::camera
