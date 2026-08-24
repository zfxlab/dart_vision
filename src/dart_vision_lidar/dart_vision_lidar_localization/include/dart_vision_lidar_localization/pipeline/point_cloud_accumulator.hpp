#ifndef DART_VISION_LIDAR_LOCALIZATION_PIPELINE_POINT_CLOUD_ACCUMULATOR_HPP
#define DART_VISION_LIDAR_LOCALIZATION_PIPELINE_POINT_CLOUD_ACCUMULATOR_HPP

#include <chrono>
#include <cstddef>
#include <deque>
#include <optional>

#include "dart_vision_lidar_localization/core/cloud_utils.hpp"

namespace dart_vision::lidar {

using CloudTimestamp = std::chrono::nanoseconds;

enum class TimestampRegressionPolicy {
    kClearAndAccept,
    kRejectFrame,
};

enum class AddFrameStatus {
    kAccepted,
    kAcceptedAfterTimeReset,
    kRejectedTimestampRegression,
};

struct PointCloudAccumulatorConfig {
    CloudTimestamp time_window{std::chrono::milliseconds{500}};
    std::size_t max_frames{10U};
    std::size_t max_points{500000U};
    TimestampRegressionPolicy timestamp_regression_policy{
        TimestampRegressionPolicy::kClearAndAccept};

    [[nodiscard]] bool isValid() const noexcept;
};

struct AddFrameResult {
    AddFrameStatus status{AddFrameStatus::kAccepted};
    std::size_t input_points{0U};
    std::size_t stored_input_points{0U};
    std::size_t evicted_frames{0U};
    std::size_t evicted_points{0U};
    std::size_t frame_count{0U};
    std::size_t total_points{0U};

    [[nodiscard]] bool accepted() const noexcept {
        return status != AddFrameStatus::kRejectedTimestampRegression;
    }
};

/**
 * @brief Chronological, bounded multi-frame point cloud accumulator.
 *
 * Frames older than time_window relative to the newest accepted timestamp are
 * removed. Oldest complete frames are then evicted until both max_frames and
 * max_points are satisfied. A single oversized frame is deterministically
 * truncated to its first max_points points, so the hard point limit is never
 * exceeded. Callers must transform every input cloud into one common frame
 * before adding it; accumulatedCloud() keeps the newest retained PCL header.
 */
class PointCloudAccumulator {
public:
    explicit PointCloudAccumulator(PointCloudAccumulatorConfig config);

    [[nodiscard]] AddFrameResult addFrame(const PointCloud& cloud, CloudTimestamp timestamp);

    /// Concatenate retained frames from oldest to newest.
    [[nodiscard]] PointCloud accumulatedCloud() const;

    void clear() noexcept;

    [[nodiscard]] const PointCloudAccumulatorConfig& config() const noexcept;
    [[nodiscard]] std::size_t frameCount() const noexcept;
    [[nodiscard]] std::size_t pointCount() const noexcept;
    [[nodiscard]] std::optional<CloudTimestamp> latestTimestamp() const noexcept;

private:
    struct StampedCloud {
        CloudTimestamp timestamp;
        PointCloud cloud;
    };

    void evictFront(AddFrameResult& result);
    void evictOutsideTimeWindow(CloudTimestamp newest_timestamp, AddFrameResult& result);

    PointCloudAccumulatorConfig config_;
    std::deque<StampedCloud> frames_;
    std::size_t point_count_{0U};
    std::optional<CloudTimestamp> latest_timestamp_;
};

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_LOCALIZATION_PIPELINE_POINT_CLOUD_ACCUMULATOR_HPP
