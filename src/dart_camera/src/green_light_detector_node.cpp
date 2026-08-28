#include "dart_camera/green_light_detector_node.hpp"

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <utility>

#include "dart_camera/observation_status.hpp"

namespace dart_vision::camera {
namespace {
rcl_interfaces::msg::SetParametersResult parameterFailure(const std::string& reason) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
}
} // namespace

GreenLightDetectorNode::GreenLightDetectorNode(const rclcpp::NodeOptions& options)
    : Node("green_light_detector", options) {
    declareParameters();
    green_light_detector_config_ = readGreenLightDetectorConfig();
    if (!green_light_detector_config_.isConfigValid()) {
        throw std::invalid_argument("Initial green-light detector parameters are invalid");
    }
    green_light_detector_ = std::make_shared<GreenLightDetector>(green_light_detector_config_);

    image_topic_ = get_parameter("image_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    observation_topic_ = get_parameter("observation_topic").as_string();
    debug_mask_topic_ = get_parameter("debug_mask_topic").as_string();
    publish_debug_mask_ = get_parameter("publish_debug_mask").as_bool();

    observation_publisher_ = create_publisher<dart_interfaces::msg::CameraObservation>(
        observation_topic_, rclcpp::SensorDataQoS());
    debug_mask_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(debug_mask_topic_, rclcpp::SensorDataQoS());
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&GreenLightDetectorNode::imageCallback, this, std::placeholders::_1));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&GreenLightDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    parameter_callback_ = add_on_set_parameters_callback(
        std::bind(&GreenLightDetectorNode::onParametersChanged, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Green-light detector listening on '%s', publishing observations on '%s'",
                image_topic_.c_str(),
                observation_topic_.c_str());
}

void GreenLightDetectorNode::declareParameters() {
    const GreenLightDetectorConfig defaults;
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "Loaded at startup; restart the node to change this parameter";

    declare_parameter<std::string>("image_topic", "image_raw", read_only);
    declare_parameter<std::string>("camera_info_topic", "camera_info", read_only);
    declare_parameter<std::string>("observation_topic", "observation", read_only);
    declare_parameter<std::string>("debug_mask_topic", "~/debug/mask", read_only);
    declare_parameter<bool>("publish_debug_mask", false);

    declare_parameter<double>("segmentation.min_hue", defaults.min_hue);
    declare_parameter<double>("segmentation.max_hue", defaults.max_hue);
    declare_parameter<double>("segmentation.min_saturation", defaults.min_saturation);
    declare_parameter<double>("segmentation.min_value", defaults.min_value);
    declare_parameter<double>("segmentation.min_green_excess", defaults.min_green_excess);

    declare_parameter<double>("geometry.min_radius_px", defaults.min_radius_px);
    declare_parameter<double>("geometry.max_radius_px", defaults.max_radius_px);
    declare_parameter<double>("geometry.min_circularity", defaults.min_circularity);
    declare_parameter<double>("geometry.max_aspect_ratio", defaults.max_aspect_ratio);
    declare_parameter<double>("geometry.min_fill_ratio", defaults.min_fill_ratio);

    declare_parameter<double>("photometry.min_inner_brightness", defaults.min_inner_brightness);
    declare_parameter<double>("photometry.min_contrast_ratio", defaults.min_contrast_ratio);

    declare_parameter<int>("mask_cleanup.close_kernel_size", defaults.cleanup.close_kernel_size);
    declare_parameter<int>("mask_cleanup.close_iterations", defaults.cleanup.close_iterations);
    declare_parameter<int>("mask_cleanup.open_kernel_size", defaults.cleanup.open_kernel_size);
    declare_parameter<int>("mask_cleanup.open_iterations", defaults.cleanup.open_iterations);
    declare_parameter<bool>("mask_cleanup.fill_holes", defaults.cleanup.fill_holes);
}

GreenLightDetectorConfig GreenLightDetectorNode::readGreenLightDetectorConfig() const {
    GreenLightDetectorConfig config;
    config.min_hue = get_parameter("segmentation.min_hue").as_double();
    config.max_hue = get_parameter("segmentation.max_hue").as_double();
    config.min_saturation = get_parameter("segmentation.min_saturation").as_double();
    config.min_value = get_parameter("segmentation.min_value").as_double();
    config.min_green_excess = get_parameter("segmentation.min_green_excess").as_double();
    config.min_radius_px = get_parameter("geometry.min_radius_px").as_double();
    config.max_radius_px = get_parameter("geometry.max_radius_px").as_double();
    config.min_circularity = get_parameter("geometry.min_circularity").as_double();
    config.max_aspect_ratio = get_parameter("geometry.max_aspect_ratio").as_double();
    config.min_fill_ratio = get_parameter("geometry.min_fill_ratio").as_double();
    config.min_inner_brightness = get_parameter("photometry.min_inner_brightness").as_double();
    config.min_contrast_ratio = get_parameter("photometry.min_contrast_ratio").as_double();
    config.cleanup.close_kernel_size =
        static_cast<int>(get_parameter("mask_cleanup.close_kernel_size").as_int());
    config.cleanup.close_iterations =
        static_cast<int>(get_parameter("mask_cleanup.close_iterations").as_int());
    config.cleanup.open_kernel_size =
        static_cast<int>(get_parameter("mask_cleanup.open_kernel_size").as_int());
    config.cleanup.open_iterations =
        static_cast<int>(get_parameter("mask_cleanup.open_iterations").as_int());
    config.cleanup.fill_holes = get_parameter("mask_cleanup.fill_holes").as_bool();
    return config;
}

rcl_interfaces::msg::SetParametersResult
GreenLightDetectorNode::onParametersChanged(const std::vector<rclcpp::Parameter>& parameters) {
    GreenLightDetectorConfig updated;
    bool green_light_detector_config_changed = false;
    bool updated_publish_debug_mask = false;
    {
        std::lock_guard<std::mutex> lock(green_light_detector_mutex_);
        updated = green_light_detector_config_;
        updated_publish_debug_mask = publish_debug_mask_;
    }

    try {
        for (const auto& parameter : parameters) {
            const std::string& name = parameter.get_name();
            if (name == "publish_debug_mask") {
                updated_publish_debug_mask = parameter.as_bool();
                continue;
            }
            if (name == "image_topic" || name == "camera_info_topic" ||
                name == "observation_topic" || name == "debug_mask_topic") {
                return parameterFailure(name + " cannot be changed while the node is running");
            }

// clang-format off
            #define UPDATE_DOUBLE(field, parameter_name)  \
                if (name == parameter_name) {             \
                    updated.field = parameter.as_double(); \
                    green_light_detector_config_changed = true; \
                    continue;                             \
                }
            UPDATE_DOUBLE(min_hue, "segmentation.min_hue")
            UPDATE_DOUBLE(max_hue, "segmentation.max_hue")
            UPDATE_DOUBLE(min_saturation, "segmentation.min_saturation")
            UPDATE_DOUBLE(min_value, "segmentation.min_value")
            UPDATE_DOUBLE(min_green_excess, "segmentation.min_green_excess")
            UPDATE_DOUBLE(min_radius_px, "geometry.min_radius_px")
            UPDATE_DOUBLE(max_radius_px, "geometry.max_radius_px")
            UPDATE_DOUBLE(min_circularity, "geometry.min_circularity")
            UPDATE_DOUBLE(max_aspect_ratio, "geometry.max_aspect_ratio")
            UPDATE_DOUBLE(min_fill_ratio, "geometry.min_fill_ratio")
            UPDATE_DOUBLE(min_inner_brightness, "photometry.min_inner_brightness")
            UPDATE_DOUBLE(min_contrast_ratio, "photometry.min_contrast_ratio")
            #undef UPDATE_DOUBLE
            // clang-format on

            if (name == "mask_cleanup.close_kernel_size") {
                updated.cleanup.close_kernel_size = static_cast<int>(parameter.as_int());
                green_light_detector_config_changed = true;
            } else if (name == "mask_cleanup.close_iterations") {
                updated.cleanup.close_iterations = static_cast<int>(parameter.as_int());
                green_light_detector_config_changed = true;
            } else if (name == "mask_cleanup.open_kernel_size") {
                updated.cleanup.open_kernel_size = static_cast<int>(parameter.as_int());
                green_light_detector_config_changed = true;
            } else if (name == "mask_cleanup.open_iterations") {
                updated.cleanup.open_iterations = static_cast<int>(parameter.as_int());
                green_light_detector_config_changed = true;
            } else if (name == "mask_cleanup.fill_holes") {
                updated.cleanup.fill_holes = parameter.as_bool();
                green_light_detector_config_changed = true;
            } else {
                continue;
            }
        }
    } catch (const rclcpp::ParameterTypeException& error) {
        return parameterFailure(error.what());
    }

    if (!updated.isConfigValid()) {
        return parameterFailure("Green-light detector parameter combination is invalid");
    }

    std::shared_ptr<GreenLightDetector> updated_green_light_detector;
    if (green_light_detector_config_changed) {
        updated_green_light_detector = std::make_shared<GreenLightDetector>(updated);
    }
    {
        std::lock_guard<std::mutex> lock(green_light_detector_mutex_);
        if (updated_green_light_detector) {
            green_light_detector_config_ = updated;
            green_light_detector_ = std::move(updated_green_light_detector);
        }
        publish_debug_mask_ = updated_publish_debug_mask;
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void GreenLightDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    std::shared_ptr<GreenLightDetector> green_light_detector;
    std::shared_ptr<BearingSolver> bearing_solver;
    std::uint32_t calibration_width = 0U;
    std::uint32_t calibration_height = 0U;
    std::string calibration_frame_id;
    bool publish_debug_mask = false;
    {
        std::lock_guard<std::mutex> lock(green_light_detector_mutex_);
        green_light_detector = green_light_detector_;
        bearing_solver = bearing_solver_;
        calibration_width = calibration_width_;
        calibration_height = calibration_height_;
        calibration_frame_id = calibration_frame_id_;
        publish_debug_mask = publish_debug_mask_;
    }

    try {
        const cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image_msg, "bgr8");
        const GreenLightDetectionResult result = green_light_detector->detect(cv_image->image);
        const bool calibration_matches_image =
            bearing_solver && (calibration_width == 0U || calibration_width == image_msg->width) &&
            (calibration_height == 0U || calibration_height == image_msg->height) &&
            (calibration_frame_id.empty() || calibration_frame_id == image_msg->header.frame_id);
        std::optional<cv::Vec3d> bearing;
        if (result.target && calibration_matches_image) {
            bearing = bearing_solver->calculateUnitBearing(result.target->center_px);
        }

        publishObservation(image_msg->header, result, bearing, calibration_matches_image);

        if (publish_debug_mask) {
            debug_mask_publisher_->publish(
                *cv_bridge::CvImage(image_msg->header, "mono8", result.binary_mask).toImageMsg());
        }
    } catch (const cv_bridge::Exception& error) {
        publishStatus(image_msg->header,
                      dart_interfaces::msg::CameraObservation::STATUS_INTERNAL_ERROR);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Image conversion failed: %s", error.what());
    } catch (const std::exception& error) {
        publishStatus(image_msg->header,
                      dart_interfaces::msg::CameraObservation::STATUS_INTERNAL_ERROR);
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "Detection failed: %s", error.what());
    }
}

void GreenLightDetectorNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {
    BearingSolverConfig config;
    std::copy(camera_info_msg->k.begin(), camera_info_msg->k.end(), config.camera_matrix.begin());
    config.distortion_coefficients = camera_info_msg->d;

    if (!config.isConfigValid()) {
        {
            std::lock_guard<std::mutex> lock(green_light_detector_mutex_);
            bearing_solver_.reset();
            bearing_solver_config_.reset();
            calibration_width_ = 0U;
            calibration_height_ = 0U;
            calibration_frame_id_.clear();
        }
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Received invalid camera information");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(green_light_detector_mutex_);
        const bool unchanged =
            bearing_solver_config_ &&
            bearing_solver_config_->camera_matrix == config.camera_matrix &&
            bearing_solver_config_->distortion_coefficients == config.distortion_coefficients &&
            calibration_width_ == camera_info_msg->width &&
            calibration_height_ == camera_info_msg->height &&
            calibration_frame_id_ == camera_info_msg->header.frame_id;
        if (unchanged) {
            return;
        }
    }

    auto solver = std::make_shared<BearingSolver>(config);
    {
        std::lock_guard<std::mutex> lock(green_light_detector_mutex_);
        bearing_solver_ = std::move(solver);
        bearing_solver_config_ = std::move(config);
        calibration_width_ = camera_info_msg->width;
        calibration_height_ = camera_info_msg->height;
        calibration_frame_id_ = camera_info_msg->header.frame_id;
    }
}

void GreenLightDetectorNode::publishObservation(const std_msgs::msg::Header& header,
                                                const GreenLightDetectionResult& result,
                                                const std::optional<cv::Vec3d>& bearing,
                                                const bool calibrated) {
    dart_interfaces::msg::CameraObservation message;
    message.header = header;
    message.status_code =
        selectObservationStatus(result.target.has_value(), calibrated, bearing.has_value());
    if (bearing) {
        message.bearing.x = (*bearing)[0];
        message.bearing.y = (*bearing)[1];
        message.bearing.z = (*bearing)[2];
    }
    observation_publisher_->publish(message);
}

void GreenLightDetectorNode::publishStatus(const std_msgs::msg::Header& header,
                                           const std::uint8_t status_code) {
    dart_interfaces::msg::CameraObservation message;
    message.header = header;
    message.status_code = status_code;
    observation_publisher_->publish(message);
}

} // namespace dart_vision::camera
