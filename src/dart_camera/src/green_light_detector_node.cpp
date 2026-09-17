#include "dart_camera/green_light_detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <utility>

#include "dart_interfaces/msg/green_light_detection.hpp"

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
    detection_topic_ = get_parameter("detection_topic").as_string();
    debug_mask_topic_ = get_parameter("debug_mask_topic").as_string();
    publish_debug_mask_ = get_parameter("publish_debug_mask").as_bool();

    detection_publisher_ = create_publisher<dart_interfaces::msg::GreenLightDetection>(
        detection_topic_, rclcpp::SensorDataQoS());
    debug_mask_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(debug_mask_topic_, rclcpp::SensorDataQoS());
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&GreenLightDetectorNode::imageCallback, this, std::placeholders::_1));
    parameter_callback_ = add_on_set_parameters_callback(
        std::bind(&GreenLightDetectorNode::onParametersChanged, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Green-light detector listening on '%s', publishing observations on '%s'",
                image_topic_.c_str(),
                detection_topic_.c_str());
}

void GreenLightDetectorNode::declareParameters() {
    const GreenLightDetectorConfig defaults;
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "Loaded at startup; restart the node to change this parameter";

    declare_parameter<std::string>("image_topic", "image_raw", read_only);
    declare_parameter<std::string>("detection_topic", "detection", read_only);
    declare_parameter<std::string>("debug_mask_topic", "~/debug/mask", read_only);
    declare_parameter<bool>("publish_debug_mask", false);
    declare_parameter<double>("ambiguity_margin", 0.05, read_only);

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
            if (name == "image_topic" || name == "detection_topic" || name == "debug_mask_topic") {
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

void GreenLightDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image) {
    using Detection = dart_interfaces::msg::GreenLightDetection;
    Detection message;
    message.header = image->header;
    std::shared_ptr<GreenLightDetector> detector;
    bool debug;
    {
        std::lock_guard<std::mutex> lock(green_light_detector_mutex_);
        detector = green_light_detector_;
        debug = publish_debug_mask_;
    }
    try {
        const auto converted = cv_bridge::toCvShare(image, "bgr8");
        auto result = detector->detect(converted->image);
        auto& candidates = result.candidates;
        std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            return a.fit_score > b.fit_score;
        });
        message.status = result.contours_count == 0 ? Detection::CLOSED : Detection::NO_TARGET;
        if (candidates.size() > 1 && candidates[0].fit_score - candidates[1].fit_score <
                                         get_parameter("ambiguity_margin").as_double()) {
            message.status = Detection::NO_TARGET;
        } else if (!candidates.empty()) {
            const auto& target = candidates.front();
            message.status = Detection::DETECTED;
            message.center_u = target.center_px.x;
            message.center_v = target.center_px.y;
            message.radius_px = target.radius_px;
            message.score = target.fit_score;
        }
        if (debug)
            debug_mask_publisher_->publish(
                *cv_bridge::CvImage(image->header, "mono8", result.binary_mask).toImageMsg());
    } catch (const std::exception& e) {
        message.status = Detection::ERROR;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
    }
    detection_publisher_->publish(message);
}
} // namespace dart_vision::camera
