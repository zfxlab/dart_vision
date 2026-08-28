#include "dart_camera/bearing_solver.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <stdexcept>

namespace dart_vision::camera {

bool BearingSolverConfig::isConfigValid() const noexcept {
    for (const double value : camera_matrix) {
        if (!std::isfinite(value)) {
            return false;
        }
    }

    if (camera_matrix[0] <= 0.0 || camera_matrix[4] <= 0.0 || camera_matrix[8] == 0.0) {
        return false;
    }

    for (const double coefficient : distortion_coefficients) {
        if (!std::isfinite(coefficient)) {
            return false;
        }
    }

    return true;
}

BearingSolver::BearingSolver(const BearingSolverConfig& config) {
    if (!config.isConfigValid()) {
        throw std::invalid_argument("Invalid bearing solver configuration");
    }

    camera_matrix_ =
        cv::Mat(3, 3, CV_64F, const_cast<double*>(config.camera_matrix.data())).clone();

    if (!config.distortion_coefficients.empty()) {
        distortion_coefficients_ = cv::Mat(config.distortion_coefficients, true).reshape(1, 1);
        distortion_coefficients_.convertTo(distortion_coefficients_, CV_64F);
    }
}

std::optional<cv::Vec3d>
BearingSolver::calculateUnitBearing(const cv::Point2f& center_px) const noexcept {
    if (!std::isfinite(center_px.x) || !std::isfinite(center_px.y)) {
        return std::nullopt;
    }

    try {
        const std::vector<cv::Point2f> distorted_points{center_px};
        std::vector<cv::Point2f> normalized_points;
        cv::undistortPoints(
            distorted_points, normalized_points, camera_matrix_, distortion_coefficients_);

        if (normalized_points.size() != 1U) {
            return std::nullopt;
        }

        const double x = normalized_points.front().x;
        const double y = normalized_points.front().y;
        const double norm = std::sqrt(x * x + y * y + 1.0);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(norm) || norm <= 0.0) {
            return std::nullopt;
        }

        return cv::Vec3d{x / norm, y / norm, 1.0 / norm};
    } catch (const cv::Exception&) {
        return std::nullopt;
    }
}

} // namespace dart_vision::camera
