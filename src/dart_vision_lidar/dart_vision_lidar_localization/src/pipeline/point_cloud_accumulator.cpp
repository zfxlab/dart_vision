#include "dart_vision_lidar_localization/pipeline/point_cloud_accumulator.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace dart_vision::lidar {

bool PointCloudAccumulatorConfig::isValid() const noexcept {
    return time_window.count() >= 0 && max_frames > 0U && max_points > 0U;
}

PointCloudAccumulator::PointCloudAccumulator(PointCloudAccumulatorConfig config) : config_(config) {
    if (!config_.isValid()) {
        throw std::invalid_argument(
            "Accumulator requires a non-negative time window and positive hard limits.");
    }
}

AddFrameResult PointCloudAccumulator::addFrame(const PointCloud& cloud, CloudTimestamp timestamp) {
    AddFrameResult result;
    result.input_points = cloud.size();

    if (latest_timestamp_ && timestamp < *latest_timestamp_) {
        if (config_.timestamp_regression_policy == TimestampRegressionPolicy::kRejectFrame) {
            result.status = AddFrameStatus::kRejectedTimestampRegression;
            result.frame_count = frames_.size();
            result.total_points = point_count_;
            return result;
        }

        result.status = AddFrameStatus::kAcceptedAfterTimeReset;
        result.evicted_frames = frames_.size();
        result.evicted_points = point_count_;
        clear();
    }

    latest_timestamp_ = timestamp;
    evictOutsideTimeWindow(timestamp, result);

    PointCloud bounded_cloud;
    bounded_cloud.header = cloud.header;
    bounded_cloud.sensor_origin_ = cloud.sensor_origin_;
    bounded_cloud.sensor_orientation_ = cloud.sensor_orientation_;
    const std::size_t bounded_size = std::min(cloud.size(), config_.max_points);
    bounded_cloud.points.assign(cloud.points.begin(),
                                cloud.points.begin() + static_cast<std::ptrdiff_t>(bounded_size));
    finalizePointCloudMetadata(bounded_cloud);
    result.stored_input_points = bounded_size;

    while (!frames_.empty() && (frames_.size() >= config_.max_frames ||
                                point_count_ > config_.max_points - bounded_cloud.size())) {
        evictFront(result);
    }

    if (!bounded_cloud.empty()) {
        point_count_ += bounded_cloud.size();
        frames_.push_back(StampedCloud{timestamp, std::move(bounded_cloud)});
    }

    result.frame_count = frames_.size();
    result.total_points = point_count_;
    return result;
}

PointCloud PointCloudAccumulator::accumulatedCloud() const {
    PointCloud accumulated;
    accumulated.points.reserve(point_count_);

    for (const StampedCloud& frame : frames_) {
        accumulated.points.insert(
            accumulated.points.end(), frame.cloud.points.begin(), frame.cloud.points.end());
    }

    if (!frames_.empty()) {
        const PointCloud& newest_cloud = frames_.back().cloud;
        accumulated.header = newest_cloud.header;
        accumulated.sensor_origin_ = newest_cloud.sensor_origin_;
        accumulated.sensor_orientation_ = newest_cloud.sensor_orientation_;
    }

    finalizePointCloudMetadata(accumulated);
    return accumulated;
}

void PointCloudAccumulator::clear() noexcept {
    frames_.clear();
    point_count_ = 0U;
    latest_timestamp_.reset();
}

const PointCloudAccumulatorConfig& PointCloudAccumulator::config() const noexcept {
    return config_;
}

std::size_t PointCloudAccumulator::frameCount() const noexcept {
    return frames_.size();
}

std::size_t PointCloudAccumulator::pointCount() const noexcept {
    return point_count_;
}

std::optional<CloudTimestamp> PointCloudAccumulator::latestTimestamp() const noexcept {
    return latest_timestamp_;
}

void PointCloudAccumulator::evictFront(AddFrameResult& result) {
    result.evicted_points += frames_.front().cloud.size();
    ++result.evicted_frames;
    point_count_ -= frames_.front().cloud.size();
    frames_.pop_front();
}

void PointCloudAccumulator::evictOutsideTimeWindow(CloudTimestamp newest_timestamp,
                                                   AddFrameResult& result) {
    const std::uint64_t newest_count = static_cast<std::uint64_t>(newest_timestamp.count());
    const std::uint64_t window_count = static_cast<std::uint64_t>(config_.time_window.count());

    while (!frames_.empty()) {
        const std::uint64_t oldest_count =
            static_cast<std::uint64_t>(frames_.front().timestamp.count());
        if (newest_count - oldest_count <= window_count) {
            break;
        }
        evictFront(result);
    }
}

} // namespace dart_vision::lidar
