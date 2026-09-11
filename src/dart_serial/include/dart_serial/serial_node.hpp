#ifndef DART_SERIAL_SERIAL_NODE_HPP
#define DART_SERIAL_SERIAL_NODE_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <thread>
#include <vector>

#include "dart_interfaces/msg/aim_command.hpp"
#include "dart_interfaces/msg/controller_state.hpp"
#include "dart_serial/packet_parser.hpp"
#include "dart_serial/serial_port.hpp"

namespace dart_vision::serial {

/**
 * @brief 连接 ROS2 消息系统与纯 C++ 串口协议栈的节点。
 *
 * 节点订阅 AimCommand，将浮点物理量按固定协议通过串口发送；接收线程从串口读取字节，
 * 经过 PacketParser 分帧和 packet 解码后发布 ControllerState。SerialPort、CRC 和协议解析
 * 均保持 ROS2 无关，参数、日志、话题以及断线重连由本类负责。
 */
class SerialNode : public rclcpp::Node {
public:
    explicit SerialNode(const rclcpp::NodeOptions& options);
    ~SerialNode() override;

private:
    void declareParameters();
    [[nodiscard]] SerialConfig readSerialConfig() const;

    void receiveLoop();
    void processBufferedFrames();
    void processFrame(const std::vector<std::uint8_t>& frame);
    void sendCallback(const dart_interfaces::msg::AimCommand::ConstSharedPtr& message);

    [[nodiscard]] bool tryOpenPort();
    void closePort() noexcept;
    void requestReconnect() noexcept;

    SerialConfig serial_config_;
    std::chrono::milliseconds reconnect_interval_{1000};

    std::string send_topic_;
    std::string receive_topic_;
    std::string joint_state_topic_;
    std::string yaw_joint_name_;

    std::unique_ptr<SerialPort> serial_port_;
    PacketParser packet_parser_;

    std::atomic<double> last_command_received_{-1.0};
    rclcpp::TimerBase::SharedPtr command_watchdog_;
    std::atomic_bool running_{false};
    std::atomic_bool connected_{false};
    std::atomic_bool reconnect_requested_{false};
    std::thread receive_thread_;

    // write() 可与 read() 并行；该锁只避免 write() 与 close()/open() 同时操作描述符。
    std::mutex port_lifecycle_mutex_;

    rclcpp::Subscription<dart_interfaces::msg::AimCommand>::SharedPtr send_subscription_;
    rclcpp::Publisher<dart_interfaces::msg::ControllerState>::SharedPtr receive_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
};

} // namespace dart_vision::serial

#endif // DART_SERIAL_SERIAL_NODE_HPP
