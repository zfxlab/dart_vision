#include "dart_serial/serial_node.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "dart_serial/packet.hpp"

namespace dart_vision::serial {

SerialNode::SerialNode(const rclcpp::NodeOptions& options) : Node("serial_node", options) {
    declareParameters();
    serial_config_ = readSerialConfig();

    const std::int64_t reconnect_interval_ms = get_parameter("reconnect_interval_ms").as_int();
    if (reconnect_interval_ms <= 0 || reconnect_interval_ms > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("reconnect_interval_ms is outside the valid range");
    }
    reconnect_interval_ = std::chrono::milliseconds(reconnect_interval_ms);

    send_topic_ = get_parameter("send_topic").as_string();
    receive_topic_ = get_parameter("receive_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    yaw_joint_name_ = get_parameter("yaw_joint_name").as_string();
    if (send_topic_.empty() || receive_topic_.empty() || joint_state_topic_.empty() ||
        yaw_joint_name_.empty()) {
        throw std::invalid_argument("Serial topic and joint names must not be empty");
    }

    serial_port_ = std::make_unique<SerialPort>(serial_config_);
    receive_publisher_ = create_publisher<dart_interfaces::msg::ControllerState>(
        receive_topic_, rclcpp::SensorDataQoS());
    joint_state_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, rclcpp::SensorDataQoS());
    send_subscription_ = create_subscription<dart_interfaces::msg::AimCommand>(
        send_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&SerialNode::sendCallback, this, std::placeholders::_1));

    const double sign = get_parameter("motor_to_joint_sign").as_double();
    const double zero = get_parameter("motor_zero_rad").as_double();
    const double timeout = get_parameter("command_timeout_s").as_double();
    if ((sign != 1.0 && sign != -1.0) || !std::isfinite(zero) || !std::isfinite(timeout) || timeout <= 0.0)
        throw std::invalid_argument("Invalid motor conversion or command timeout");
    command_watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        const double age = now().seconds() - last_command_received_.load();
        if (last_command_received_.load() < 0.0 || age < 0.0 || age > get_parameter("command_timeout_s").as_double()) {
            auto invalid = std::make_shared<dart_interfaces::msg::AimCommand>();
            invalid->header.stamp = now();
            sendCallback(invalid);
        }
    });
    running_.store(true);
    receive_thread_ = std::thread(&SerialNode::receiveLoop, this);

    RCLCPP_INFO(get_logger(),
                "Serial node started: device=%s baud=%u send_topic=%s receive_topic=%s",
                serial_config_.device.c_str(),
                serial_config_.baud_rate,
                send_topic_.c_str(),
                receive_topic_.c_str());
}

SerialNode::~SerialNode() {
    running_.store(false);
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    closePort();
}

void SerialNode::declareParameters() {
    declare_parameter<std::string>("device", "/dev/ttyACM0");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<double>("motor_to_joint_sign", 1.0);
    declare_parameter<double>("motor_zero_rad", 0.0);
    declare_parameter<double>("command_timeout_s", 0.2);
    declare_parameter<int>("read_timeout_ms", 100);
    declare_parameter<int>("reconnect_interval_ms", 1000);
    declare_parameter<std::string>("send_topic", "aim_command");
    declare_parameter<std::string>("receive_topic", "controller_state");
    declare_parameter<std::string>("joint_state_topic", "joint_states");
    declare_parameter<std::string>("yaw_joint_name", "launcher_yaw_joint");
}

SerialConfig SerialNode::readSerialConfig() const {
    const std::int64_t baud_rate = get_parameter("baud_rate").as_int();
    const std::int64_t read_timeout_ms = get_parameter("read_timeout_ms").as_int();

    if (baud_rate <= 0 ||
        baud_rate > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) {
        throw std::invalid_argument("baud_rate is outside uint32 range");
    }
    if (read_timeout_ms < 0 || read_timeout_ms > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("read_timeout_ms is outside int range");
    }

    SerialConfig config;
    config.device = get_parameter("device").as_string();
    config.baud_rate = static_cast<std::uint32_t>(baud_rate);
    config.read_timeout_ms = static_cast<int>(read_timeout_ms);
    return config;
}

void SerialNode::receiveLoop() {
    std::array<std::uint8_t, 256> read_buffer{};

    while (running_.load() && rclcpp::ok()) {
        if (!connected_.load() || reconnect_requested_.exchange(false)) {
            closePort();
            packet_parser_.reset();

            if (!tryOpenPort()) {
                std::this_thread::sleep_for(reconnect_interval_);
                continue;
            }
        }

        try {
            const std::size_t bytes_read =
                serial_port_->read(read_buffer.data(), read_buffer.size());
            if (bytes_read == 0) {
                continue;
            }

            packet_parser_.append(read_buffer.data(), bytes_read);
            processBufferedFrames();
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "Serial receive failed: %s", error.what());
            requestReconnect();
        }
    }
}

void SerialNode::processBufferedFrames() {
    while (running_.load() && rclcpp::ok()) {
        ParseResult result = packet_parser_.nextFrame();

        switch (result.status) {
            case ParseStatus::kNeedMoreData:
                return;
            case ParseStatus::kFrameReady:
                processFrame(result.frame);
                break;
            case ParseStatus::kCRCError:
                RCLCPP_WARN_THROTTLE(get_logger(),
                                     *get_clock(),
                                     2000,
                                     "Received a %zu-byte frame with invalid CRC16",
                                     result.frame.size());
                break;
            case ParseStatus::kUnknownHeader:
                RCLCPP_WARN_THROTTLE(get_logger(),
                                     *get_clock(),
                                     2000,
                                     "Discarded %zu byte(s) before a valid receive header",
                                     result.frame.size());
                break;
        }
    }
}

void SerialNode::processFrame(const std::vector<std::uint8_t>& frame) {
    const auto packet = decodeReceivePacket(frame);
    if (!packet.has_value()) {
        RCLCPP_WARN(get_logger(), "A parser-approved receive frame failed packet decoding");
        return;
    }

    if (!std::isfinite(packet->yaw_rad) || !std::isfinite(packet->offset_rad)) return;
    const rclcpp::Time stamp = now();

    dart_interfaces::msg::ControllerState message;
    message.header.stamp = stamp;
    message.header.frame_id = "serial";
    message.target_id = packet->target_id;
    message.offset_rad = packet->offset_rad;
    message.yaw_rad = packet->yaw_rad;
    receive_publisher_->publish(message);

    sensor_msgs::msg::JointState state;
    state.header.stamp = stamp;
    state.name.push_back(yaw_joint_name_);
    state.position.push_back(get_parameter("motor_to_joint_sign").as_double() *
        (packet->yaw_rad - get_parameter("motor_zero_rad").as_double()));
    joint_state_publisher_->publish(state);
}

void SerialNode::sendCallback(const dart_interfaces::msg::AimCommand::ConstSharedPtr& message) {
    last_command_received_.store(now().seconds());
    if (!connected_.load()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Dropping aim command: serial port is disconnected");
        return;
    }

    try {
        SendPacket packet;
        const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
        const bool valid = age >= 0.0 && age <= get_parameter("command_timeout_s").as_double() &&
            std::isfinite(message->yaw_rad) && std::isfinite(message->distance_m) &&
            message->distance_m > 0.0F && message->state >= 1 && message->state <= 3;
        packet.state = valid ? message->state : 0;
        packet.yaw_rad = valid ? message->yaw_rad : 0.0F;
        packet.distance_m = valid ? message->distance_m : 0.0F;

        const auto frame = encodeSendPacket(packet);
        std::lock_guard<std::mutex> lock(port_lifecycle_mutex_);
        if (!connected_.load() || !serial_port_->isOpen()) {
            return;
        }
        serial_port_->write(frame.data(), frame.size());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Serial send failed: %s", error.what());
        requestReconnect();
    }
}

bool SerialNode::tryOpenPort() {
    try {
        std::lock_guard<std::mutex> lock(port_lifecycle_mutex_);
        serial_port_->open();
        connected_.store(true);
        RCLCPP_INFO(get_logger(), "Opened serial device %s", serial_config_.device.c_str());
        return true;
    } catch (const std::exception& error) {
        connected_.store(false);
        RCLCPP_WARN(get_logger(),
                    "Unable to open serial device %s: %s",
                    serial_config_.device.c_str(),
                    error.what());
        return false;
    }
}

void SerialNode::closePort() noexcept {
    std::lock_guard<std::mutex> lock(port_lifecycle_mutex_);
    connected_.store(false);
    if (serial_port_) {
        serial_port_->close();
    }
}

void SerialNode::requestReconnect() noexcept {
    connected_.store(false);
    reconnect_requested_.store(true);
}

} // namespace dart_vision::serial
