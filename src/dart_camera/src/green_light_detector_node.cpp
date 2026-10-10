#include "dart_camera/green_light_detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <functional>
#include <geometry_msgs/msg/vector3.hpp>
#include <limits>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "dart_camera/bearing_solver.hpp"
#include "dart_interfaces/msg/green_light_detection.hpp"

namespace dart_vision::camera {
namespace {
rcl_interfaces::msg::SetParametersResult parameterFailure(const std::string& reason) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = value;
    return result;
}

bool validProfileName(const std::string& name) {
    static const std::regex pattern{"[A-Za-z][A-Za-z0-9_]*"};
    return std::regex_match(name, pattern);
}

bool validAmbiguityMargin(const double value) {
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

bool endsWith(const std::string& value, const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
} // namespace

GreenLightDetectorNode::GreenLightDetectorNode(const rclcpp::NodeOptions& options)
    : Node("green_light_detector", options),
      previous_diagnostic_time_(std::chrono::steady_clock::now()) {
    declareParameters();
    loadProfiles();

    image_topic_ = get_parameter("image_topic").as_string();
    detection_topic_ = get_parameter("detection_topic").as_string();

    detection_publisher_ = create_publisher<dart_interfaces::msg::GreenLightDetection>(
        detection_topic_, rclcpp::SensorDataQoS());
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
    diagnostics_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&GreenLightDetectorNode::publishDiagnostics, this));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("camera_info_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
            latest_camera_info_ = std::move(info);
            processPendingImages();
        });
    camera_info_timer_ =
        create_wall_timer(std::chrono::milliseconds(20), [this] { processPendingImages(); });
    controller_subscription_ = create_subscription<dart_interfaces::msg::ControllerState>(
        get_parameter("controller_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&GreenLightDetectorNode::controllerCallback, this, std::placeholders::_1));
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&GreenLightDetectorNode::imageCallback, this, std::placeholders::_1));
    parameter_callback_ = add_on_set_parameters_callback(
        std::bind(&GreenLightDetectorNode::onParametersChanged, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Green-light detector listening on '%s', publishing observations on '%s' with "
                "%zu target profiles",
                image_topic_.c_str(), detection_topic_.c_str(), profiles_.size());
}

void GreenLightDetectorNode::declareParameters() {
    const GreenLightDetectorConfig defaults;
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "Loaded at startup; restart the node to change this parameter";

    declare_parameter<std::string>("image_topic", "image_raw", read_only);
    declare_parameter<std::string>("detection_topic", "detection", read_only);
    declare_parameter<std::string>("camera_info_topic", "camera_info", read_only);
    declare_parameter<std::string>("controller_topic", "/controller_state", read_only);
    const auto profile_names = declare_parameter<std::vector<std::string>>(
        "profiles.names", std::vector<std::string>{"default"}, read_only);
    if (profile_names.empty())
        throw std::invalid_argument("At least one detector profile is required");
    std::unordered_set<std::string> declared_names;
    for (const auto& profile_name : profile_names) {
        if (!validProfileName(profile_name) || !declared_names.insert(profile_name).second)
            throw std::invalid_argument("Invalid or duplicate detector profile name: " +
                                        profile_name);
        const std::string prefix = "profiles." + profile_name + ".";
        const std::vector<std::int64_t> default_modes =
            profile_name == "default" ? std::vector<std::int64_t>{0, 1, 2, 3, 4}
                                      : std::vector<std::int64_t>{};
        declare_parameter<std::vector<std::int64_t>>(prefix + "modes", default_modes, read_only);
        declare_parameter<double>(prefix + "ambiguity_margin", 0.05);
        declare_parameter<double>(prefix + "segmentation.min_hue", defaults.min_hue);
        declare_parameter<double>(prefix + "segmentation.max_hue", defaults.max_hue);
        declare_parameter<double>(prefix + "segmentation.min_saturation", defaults.min_saturation);
        declare_parameter<double>(prefix + "segmentation.min_value", defaults.min_value);
        declare_parameter<double>(prefix + "segmentation.min_green_excess",
                                  defaults.min_green_excess);
        declare_parameter<double>(prefix + "geometry.min_radius_px", defaults.min_radius_px);
        declare_parameter<double>(prefix + "geometry.max_radius_px", defaults.max_radius_px);
        declare_parameter<double>(prefix + "geometry.min_circularity", defaults.min_circularity);
        declare_parameter<double>(prefix + "geometry.max_aspect_ratio", defaults.max_aspect_ratio);
        declare_parameter<double>(prefix + "geometry.min_fill_ratio", defaults.min_fill_ratio);
        declare_parameter<double>(prefix + "photometry.min_inner_brightness",
                                  defaults.min_inner_brightness);
        declare_parameter<double>(prefix + "photometry.min_contrast_ratio",
                                  defaults.min_contrast_ratio);
        declare_parameter<int>(prefix + "mask_cleanup.close_kernel_size",
                               defaults.cleanup.close_kernel_size);
        declare_parameter<int>(prefix + "mask_cleanup.close_iterations",
                               defaults.cleanup.close_iterations);
        declare_parameter<int>(prefix + "mask_cleanup.open_kernel_size",
                               defaults.cleanup.open_kernel_size);
        declare_parameter<int>(prefix + "mask_cleanup.open_iterations",
                               defaults.cleanup.open_iterations);
        declare_parameter<bool>(prefix + "mask_cleanup.fill_holes", defaults.cleanup.fill_holes);
    }
}

GreenLightDetectorConfig
GreenLightDetectorNode::readGreenLightDetectorConfig(const std::string& prefix) const {
    GreenLightDetectorConfig config;
    config.min_hue = get_parameter(prefix + "segmentation.min_hue").as_double();
    config.max_hue = get_parameter(prefix + "segmentation.max_hue").as_double();
    config.min_saturation = get_parameter(prefix + "segmentation.min_saturation").as_double();
    config.min_value = get_parameter(prefix + "segmentation.min_value").as_double();
    config.min_green_excess = get_parameter(prefix + "segmentation.min_green_excess").as_double();
    config.min_radius_px = get_parameter(prefix + "geometry.min_radius_px").as_double();
    config.max_radius_px = get_parameter(prefix + "geometry.max_radius_px").as_double();
    config.min_circularity = get_parameter(prefix + "geometry.min_circularity").as_double();
    config.max_aspect_ratio = get_parameter(prefix + "geometry.max_aspect_ratio").as_double();
    config.min_fill_ratio = get_parameter(prefix + "geometry.min_fill_ratio").as_double();
    config.min_inner_brightness =
        get_parameter(prefix + "photometry.min_inner_brightness").as_double();
    config.min_contrast_ratio = get_parameter(prefix + "photometry.min_contrast_ratio").as_double();
    config.cleanup.close_kernel_size =
        static_cast<int>(get_parameter(prefix + "mask_cleanup.close_kernel_size").as_int());
    config.cleanup.close_iterations =
        static_cast<int>(get_parameter(prefix + "mask_cleanup.close_iterations").as_int());
    config.cleanup.open_kernel_size =
        static_cast<int>(get_parameter(prefix + "mask_cleanup.open_kernel_size").as_int());
    config.cleanup.open_iterations =
        static_cast<int>(get_parameter(prefix + "mask_cleanup.open_iterations").as_int());
    config.cleanup.fill_holes = get_parameter(prefix + "mask_cleanup.fill_holes").as_bool();
    return config;
}

void GreenLightDetectorNode::loadProfiles() {
    const auto profile_names = get_parameter("profiles.names").as_string_array();
    for (const auto& profile_name : profile_names) {
        const std::string prefix = "profiles." + profile_name + ".";
        DetectorProfile profile;
        profile.config = readGreenLightDetectorConfig(prefix);
        profile.ambiguity_margin = get_parameter(prefix + "ambiguity_margin").as_double();
        if (!profile.config.isConfigValid() || !validAmbiguityMargin(profile.ambiguity_margin))
            throw std::invalid_argument("Invalid green-light detector profile: " + profile_name);
        profile.detector = std::make_shared<GreenLightDetector>(profile.config);
        profiles_.emplace(profile_name, std::move(profile));

        const auto modes = get_parameter(prefix + "modes").as_integer_array();
        if (modes.empty())
            throw std::invalid_argument("Detector profile has no target modes: " + profile_name);
        std::unordered_set<std::int64_t> profile_modes;
        for (const auto mode : modes) {
            if (mode < 0 || mode > std::numeric_limits<std::uint8_t>::max() ||
                !profile_modes.insert(mode).second)
                throw std::invalid_argument("Invalid or duplicate target mode in profile: " +
                                            profile_name);
            if (!mode_profiles_.emplace(static_cast<std::uint8_t>(mode), profile_name).second)
                throw std::invalid_argument("Target mode belongs to multiple detector profiles: " +
                                            std::to_string(mode));
        }
    }
}

rcl_interfaces::msg::SetParametersResult
GreenLightDetectorNode::onParametersChanged(const std::vector<rclcpp::Parameter>& parameters) {
    std::unordered_map<std::string, DetectorProfile> updated_profiles;
    {
        std::lock_guard<std::mutex> lock(profiles_mutex_);
        updated_profiles = profiles_;
    }
    std::unordered_set<std::string> changed_profiles;

    try {
        for (const auto& parameter : parameters) {
            const std::string& name = parameter.get_name();
            if (name == "image_topic" || name == "detection_topic" || name == "camera_info_topic" ||
                name == "controller_topic" || name == "profiles.names" ||
                (name.rfind("profiles.", 0) == 0 && endsWith(name, ".modes"))) {
                return parameterFailure(name + " cannot be changed while the node is running");
            }

            for (auto& [profile_name, profile] : updated_profiles) {
                const std::string prefix = "profiles." + profile_name + ".";
                if (name.rfind(prefix, 0) != 0)
                    continue;
                const std::string field = name.substr(prefix.size());

                // clang-format off
                #define UPDATE_DOUBLE(member, parameter_name) \
                    if (field == parameter_name) {             \
                        profile.config.member = parameter.as_double(); \
                        changed_profiles.insert(profile_name); \
                        break;                                 \
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

                if (field == "ambiguity_margin") {
                    profile.ambiguity_margin = parameter.as_double();
                } else if (field == "mask_cleanup.close_kernel_size") {
                    profile.config.cleanup.close_kernel_size = static_cast<int>(parameter.as_int());
                } else if (field == "mask_cleanup.close_iterations") {
                    profile.config.cleanup.close_iterations = static_cast<int>(parameter.as_int());
                } else if (field == "mask_cleanup.open_kernel_size") {
                    profile.config.cleanup.open_kernel_size = static_cast<int>(parameter.as_int());
                } else if (field == "mask_cleanup.open_iterations") {
                    profile.config.cleanup.open_iterations = static_cast<int>(parameter.as_int());
                } else if (field == "mask_cleanup.fill_holes") {
                    profile.config.cleanup.fill_holes = parameter.as_bool();
                } else {
                    break;
                }
                changed_profiles.insert(profile_name);
                break;
            }
        }
    } catch (const rclcpp::ParameterTypeException& error) {
        return parameterFailure(error.what());
    }

    for (const auto& profile_name : changed_profiles) {
        auto& profile = updated_profiles.at(profile_name);
        if (!profile.config.isConfigValid() || !validAmbiguityMargin(profile.ambiguity_margin))
            return parameterFailure("Green-light detector profile is invalid: " + profile_name);
        profile.detector = std::make_shared<GreenLightDetector>(profile.config);
    }
    if (!changed_profiles.empty()) {
        std::lock_guard<std::mutex> lock(profiles_mutex_);
        profiles_ = std::move(updated_profiles);
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void GreenLightDetectorNode::controllerCallback(
    const dart_interfaces::msg::ControllerState::ConstSharedPtr& controller) {
    const auto selected = mode_profiles_.find(controller->target_mode);
    if (selected == mode_profiles_.end()) {
        {
            std::lock_guard<std::mutex> lock(profiles_mutex_);
            active_profile_.reset();
        }
        pending_images_.clear();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "No green-light detector profile for target mode %u",
                             static_cast<unsigned int>(controller->target_mode));
        return;
    }

    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(profiles_mutex_);
        changed = !active_profile_ || *active_profile_ != selected->second;
        active_profile_ = selected->second;
    }
    if (changed) {
        pending_images_.clear();
        RCLCPP_INFO(get_logger(), "Selected green-light detector profile '%s' for target mode %u",
                    selected->second.c_str(), static_cast<unsigned int>(controller->target_mode));
    }
}

void GreenLightDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image) {
    {
        std::lock_guard<std::mutex> lock(diagnostic_mutex_);
        ++diagnostic_statistics_.received_total;
        ++diagnostic_statistics_.received_interval;
    }
    pending_images_.push_back({image, std::chrono::steady_clock::now()});
    processPendingImages();
}

void GreenLightDetectorNode::processPendingImages() {
    while (!pending_images_.empty()) {
        const auto& pending = pending_images_.front();
        // Calibration is reused across frames; only wait for the initial CameraInfo.
        if (!latest_camera_info_ && pending_images_.size() <= 10 &&
            std::chrono::steady_clock::now() - pending.received < std::chrono::milliseconds(200))
            return;
        const auto image = pending.image;
        const auto info = latest_camera_info_;
        pending_images_.pop_front();
        processImage(image, info);
    }
}

void GreenLightDetectorNode::processImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
    using Detection = dart_interfaces::msg::GreenLightDetection;
    const auto processing_start = std::chrono::steady_clock::now();
    Detection message;
    message.header = image->header;
    std::shared_ptr<GreenLightDetector> detector;
    double ambiguity_margin = 0.0;
    {
        std::lock_guard<std::mutex> lock(profiles_mutex_);
        if (active_profile_) {
            const auto profile = profiles_.find(*active_profile_);
            if (profile != profiles_.end()) {
                detector = profile->second.detector;
                ambiguity_margin = profile->second.ambiguity_margin;
            }
        }
    }
    try {
        if (!detector)
            throw std::runtime_error("Waiting for a supported controller target mode");
        const auto converted = cv_bridge::toCvShare(image, "bgr8");
        auto result = detector->detect(converted->image);
        auto& candidates = result.candidates;
        std::sort(candidates.begin(), candidates.end(),
                  [](const auto& a, const auto& b) { return a.fit_score > b.fit_score; });
        message.status = result.contours_count == 0 ? Detection::CLOSED : Detection::NO_TARGET;
        if (candidates.size() > 1 &&
            candidates[0].fit_score - candidates[1].fit_score < ambiguity_margin) {
            message.status = Detection::NO_TARGET;
        } else if (!candidates.empty()) {
            const auto& target = candidates.front();
            if (!info || info->header.frame_id.empty() ||
                info->header.frame_id != image->header.frame_id || info->width != image->width ||
                info->height != image->height || info->distortion_model != "plumb_bob" ||
                info->binning_x > 1 || info->binning_y > 1 || info->roi.x_offset ||
                info->roi.y_offset || (info->roi.width && info->roi.width != image->width) ||
                (info->roi.height && info->roi.height != image->height) ||
                !std::isfinite(target.center_px.x) || !std::isfinite(target.center_px.y) ||
                target.center_px.x < 0 || target.center_px.y < 0 ||
                target.center_px.x >= image->width || target.center_px.y >= image->height)
                throw std::runtime_error("Missing or incompatible CameraInfo for target image");
            BearingSolverConfig config;
            config.camera_matrix = info->k;
            config.distortion_coefficients = info->d;
            const auto ray = BearingSolver(config).calculateUnitBearing(target.center_px);
            if (!ray)
                throw std::runtime_error("Cannot calculate target unit ray");
            message.status = Detection::DETECTED;
            message.unit_ray.x = (*ray)[0];
            message.unit_ray.y = (*ray)[1];
            message.unit_ray.z = (*ray)[2];
            message.score = target.fit_score;
        }
    } catch (const std::exception& e) {
        message.status = Detection::ERROR;
        message.unit_ray = geometry_msgs::msg::Vector3{};
        message.score = 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
    }
    detection_publisher_->publish(message);

    const auto processing_end = std::chrono::steady_clock::now();
    const auto processing_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(processing_end - processing_start);
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    auto& statistics = diagnostic_statistics_;
    ++statistics.processed_total;
    ++statistics.processed_interval;
    statistics.processing_time_interval += processing_time;
    statistics.max_processing_time_interval =
        std::max(statistics.max_processing_time_interval, processing_time);
    statistics.last_processed_time = processing_end;
    switch (message.status) {
    case Detection::DETECTED:
        ++statistics.detected_total;
        ++statistics.detected_interval;
        break;
    case Detection::CLOSED:
        ++statistics.closed_total;
        break;
    case Detection::NO_TARGET:
        ++statistics.no_target_total;
        break;
    case Detection::ERROR:
    default:
        ++statistics.errors_total;
        ++statistics.errors_interval;
        break;
    }
}

void GreenLightDetectorNode::publishDiagnostics() {
    const auto current_time = std::chrono::steady_clock::now();
    const double interval_seconds =
        std::chrono::duration<double>(current_time - previous_diagnostic_time_).count();
    previous_diagnostic_time_ = current_time;

    DiagnosticStatistics statistics;
    {
        std::lock_guard<std::mutex> lock(diagnostic_mutex_);
        statistics = diagnostic_statistics_;
        diagnostic_statistics_.received_interval = 0;
        diagnostic_statistics_.processed_interval = 0;
        diagnostic_statistics_.detected_interval = 0;
        diagnostic_statistics_.errors_interval = 0;
        diagnostic_statistics_.processing_time_interval = std::chrono::nanoseconds::zero();
        diagnostic_statistics_.max_processing_time_interval = std::chrono::nanoseconds::zero();
    }

    const double input_fps =
        interval_seconds > 0.0
            ? static_cast<double>(statistics.received_interval) / interval_seconds
            : 0.0;
    const double processed_fps =
        interval_seconds > 0.0
            ? static_cast<double>(statistics.processed_interval) / interval_seconds
            : 0.0;
    const double detected_fps =
        interval_seconds > 0.0
            ? static_cast<double>(statistics.detected_interval) / interval_seconds
            : 0.0;
    const double detection_ratio = statistics.processed_interval > 0
                                       ? static_cast<double>(statistics.detected_interval) /
                                             static_cast<double>(statistics.processed_interval)
                                       : 0.0;
    const double average_processing_ms =
        statistics.processed_interval > 0
            ? std::chrono::duration<double, std::milli>(statistics.processing_time_interval)
                      .count() /
                  static_cast<double>(statistics.processed_interval)
            : 0.0;
    const double max_processing_ms =
        std::chrono::duration<double, std::milli>(statistics.max_processing_time_interval).count();
    const double last_processed_age_seconds =
        statistics.last_processed_time == std::chrono::steady_clock::time_point{}
            ? -1.0
            : std::chrono::duration<double>(current_time - statistics.last_processed_time).count();
    std::string active_profile = "none";
    {
        std::lock_guard<std::mutex> lock(profiles_mutex_);
        if (active_profile_)
            active_profile = *active_profile_;
    }

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_fully_qualified_name()) + ": green_light_detector";
    status.hardware_id = "none";
    if (statistics.processed_interval == 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "no images processed";
    } else if (statistics.errors_interval > 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "image processing errors";
    } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "processing";
    }
    status.values.push_back(keyValue("input_fps", std::to_string(input_fps)));
    status.values.push_back(keyValue("processed_fps", std::to_string(processed_fps)));
    status.values.push_back(keyValue("detected_fps", std::to_string(detected_fps)));
    status.values.push_back(keyValue("detection_ratio", std::to_string(detection_ratio)));
    status.values.push_back(
        keyValue("average_processing_ms", std::to_string(average_processing_ms)));
    status.values.push_back(keyValue("max_processing_ms", std::to_string(max_processing_ms)));
    status.values.push_back(
        keyValue("last_processed_age_sec", std::to_string(last_processed_age_seconds)));
    status.values.push_back(keyValue("active_profile", active_profile));
    status.values.push_back(
        keyValue("images_received_total", std::to_string(statistics.received_total)));
    status.values.push_back(
        keyValue("frames_processed_total", std::to_string(statistics.processed_total)));
    status.values.push_back(keyValue("detected_total", std::to_string(statistics.detected_total)));
    status.values.push_back(keyValue("closed_total", std::to_string(statistics.closed_total)));
    status.values.push_back(
        keyValue("no_target_total", std::to_string(statistics.no_target_total)));
    status.values.push_back(keyValue("errors_total", std::to_string(statistics.errors_total)));
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
}
} // namespace dart_vision::camera

RCLCPP_COMPONENTS_REGISTER_NODE(dart_vision::camera::GreenLightDetectorNode)
