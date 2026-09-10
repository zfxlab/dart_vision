#include "dart_lidar_localization/ros/base_calibration_node.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <stdexcept>
#include <tf2_eigen/tf2_eigen.hpp>

#include "dart_lidar_localization/io/model_loader.hpp"

namespace dart_vision::lidar::localization {
BaseCalibrationNode::BaseCalibrationNode(const rclcpp::NodeOptions& options)
    : Node("base_calibration", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    model_path_ = declare_parameter<std::string>("model_path", "");
    parameters_.coarse_enabled = declare_parameter<bool>("coarse_enabled", false);
    parameters_.dof_mode = parseDofMode(declare_parameter<std::string>("dof", "xyzyaw"));
    parameters_.max_total_time_ms = 2000.0;
    const auto minimum_input = declare_parameter<std::int64_t>("min_input_points", 500);
    const auto minimum_pairs =
        declare_parameter<std::int64_t>("validation.min_correspondences", 350);
    if (minimum_input <= 0 || minimum_pairs <= 0) {
        throw std::invalid_argument("calibration point-count thresholds must be positive");
    }
    parameters_.min_input_points = static_cast<std::size_t>(minimum_input);
    parameters_.validation.min_correspondences = static_cast<std::size_t>(minimum_pairs);
    parameters_.validation.correspondence_distance_m = 0.04;
    parameters_.validation.max_rmse_m = 0.025;
    parameters_.validation.min_overlap_ratio = 0.35;
    parameters_.validation.enforce_initial_deviation_limits = false;
    parameters_.ndt_levels = {{0.10, 0.12, 0.10, 0.001, 60}, {0.05, 0.06, 0.05, 0.0005, 40}};
    parameters_.gicp_levels = {{0.03, 0.12, 1.0e-6, 1.0e-6, 40}, {0.015, 0.04, 1.0e-6, 1.0e-6, 60}};
    save_once_ = declare_parameter<bool>("output.save_once", true);
    output_directory_ = declare_parameter<std::string>("output.directory", "");
    bag_name_ = declare_parameter<std::string>("output.bag_name", "");
    if (output_directory_.empty()) {
        throw std::invalid_argument("output.directory is required for calibration export");
    }
    registrar_ = std::make_unique<BaseRegistrar>(parameters_, ModelLoader::load(model_path_));
    candidate_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "lidar/calibration/candidate_aligned", rclcpp::QoS(1).best_effort());
    aligned_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "lidar/calibration/aligned", rclcpp::QoS(1).best_effort());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/accumulated",
        rclcpp::QoS(1).best_effort(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { cloudCallback(message); });
    export_service_ = create_service<std_srvs::srv::Trigger>(
        "lidar/calibration/export_candidate",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            if (!latest_candidate_) {
                response->success = false;
                response->message = "No complete candidate from the latest processed input";
                return;
            }
            try {
                response->message = exportRecord(*latest_candidate_);
                response->success = true;
            } catch (const std::exception& error) {
                response->success = false;
                response->message = error.what();
            }
        });
    RCLCPP_INFO(get_logger(),
                "Base calibration only; coarse=%s, auto export once=%s",
                parameters_.coarse_enabled ? "on" : "off",
                save_once_ ? "on" : "off");
}

void BaseCalibrationNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
    try {
        const auto stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
        if (stamp_ns <= 0 || message->header.frame_id != "base_nominal_link") {
            latest_candidate_.reset();
            confirmation_.reset();
            throw std::runtime_error("expected positive timestamp in base_nominal_link");
        }
        if (stamp_ns == last_stamp_ns_) {
            return;
        }
        if (stamp_ns < last_stamp_ns_) {
            confirmation_.reset();
            reference_from_base_.reset();
            // 自动导出锁存不重置：bag循环播放仍然属于同一次进程运行。
        }
        last_stamp_ns_ = stamp_ns;
        latest_candidate_.reset();
        if (!reference_from_base_) {
            const auto transform =
                tf_buffer_.lookupTransform("base_nominal_link", "base_link", tf2::TimePointZero);
            reference_from_base_ = tf2::transformToEigen(transform.transform);
        }
        PointCloud::Ptr observation(new PointCloud);
        pcl::fromROSMsg(*message, *observation);
        const auto result = registrar_->align(observation, reference_from_base_->inverse());
        const bool confirmed = confirmation_.observe(result, stamp_ns);
        ++measurement_id_;
        if (result.hasCandidate()) {
            PointCloud aligned;
            pcl::transformPointCloud(
                *observation, aligned, result.target_from_source.matrix().cast<float>());
            sensor_msgs::msg::PointCloud2 debug;
            pcl::toROSMsg(aligned, debug);
            debug.header = message->header;
            debug.header.frame_id = "base_link";
            candidate_publisher_->publish(debug);
            if (result.success()) {
                aligned_publisher_->publish(debug);
            }
            LocalizationRecord record;
            record.profile = "calibration";
            record.measurement_id = measurement_id_;
            record.source_stamp_sec = message->header.stamp.sec;
            record.source_stamp_nanosec = message->header.stamp.nanosec;
            record.source_frame = message->header.frame_id;
            record.reference_frame = "base_nominal_link";
            record.base_frame = "base_link";
            record.base_model_path = ModelLoader::resolvePath(model_path_);
            record.bag_name = bag_name_;
            record.confirmed = confirmed;
            record.result = result;
            record.base_parameters = parameters_;
            latest_candidate_ = record;
            if (confirmed && save_once_ && !saved_) {
                const auto path = exportRecord(record);
                saved_ = true; // 仅在成功写入后锁存。
                RCLCPP_INFO(get_logger(),
                            "Calibration confirmed and exported once: %s; "
                            "review base.six_dof and edit site manually",
                            path.c_str());
            }
        }
        if (!result.success()) {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 2000,
                                 "Base calibration unavailable: %s",
                                 result.message.c_str());
        }
    } catch (const std::exception& error) {
        confirmation_.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Calibration: %s", error.what());
    }
}

std::string BaseCalibrationNode::exportRecord(const LocalizationRecord& source) {
    auto record = source;
    const auto wall_now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(wall_now);
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(wall_now.time_since_epoch()).count() %
        1000;
    std::tm local{};
    localtime_r(&time, &local);
    std::ostringstream iso, filename;
    iso << std::put_time(&local, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(3) << std::setfill('0')
        << ms << std::put_time(&local, "%z");
    filename << std::put_time(&local, "%Y%m%d_%H%M%S") << '_' << std::setw(3) << std::setfill('0')
             << ms;
    record.generated_at = iso.str();
    record.filename_timestamp = filename.str();
    return ResultWriter::write(output_directory_, "base", record);
}
} // namespace dart_vision::lidar::localization
