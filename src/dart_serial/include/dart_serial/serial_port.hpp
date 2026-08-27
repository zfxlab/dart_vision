#ifndef DART_SERIAL_SERIAL_PORT_HPP
#define DART_SERIAL_SERIAL_PORT_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace dart_vision::serial {

/**
 * @brief 串口设备配置。
 *
 * 当前通信格式固定为 8N1：8 个数据位、无奇偶校验、1 个停止位，并关闭软硬件流控。
 */
struct SerialConfig {
    std::string device{"/dev/ttyACM0"}; ///< Linux 串口设备路径。
    std::uint32_t baud_rate{115200};     ///< 通信波特率。

    /**
     * @brief 单次 read() 等待数据的最长时间，单位为毫秒。
     *
     * 设为 0 时立即返回；大于 0 时最多等待指定时间。有限超时可以让接收线程周期性检查
     * 退出标志，避免程序关闭时永久阻塞在串口读取上。
     */
    int read_timeout_ms{100};
};

/**
 * @brief Linux POSIX 串口的轻量 RAII 封装。
 *
 * SerialPort 只负责设备的打开、配置和原始字节收发，不依赖 ROS2，也不处理协议帧、CRC
 * 或自动重连。协议分帧由 PacketParser 负责，重连策略由上层 SerialNode 负责。
 *
 * 对象拥有一个文件描述符：析构时会自动关闭。为避免同一个描述符被重复关闭，本类禁止
 * 拷贝。一个线程读取、另一个线程写入是常见用法；但 open()、close() 与正在进行的
 * read()/write() 不应并发调用，应由上层统一管理生命周期。
 */
class SerialPort {
public:
    /**
     * @brief 保存并验证串口配置，但不立即打开设备。
     *
     * @throws std::invalid_argument 设备路径为空、波特率不支持或读取超时为负数。
     */
    explicit SerialPort(const SerialConfig& config);

    /// 自动关闭当前持有的串口文件描述符。
    ~SerialPort() noexcept;

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&&) = delete;
    SerialPort& operator=(SerialPort&&) = delete;

    /**
     * @brief 打开设备并配置为原始字节模式、8N1、无流控。
     *
     * 如果串口已经打开则直接返回。
     *
     * @throws std::system_error 打开设备或设置 termios 属性失败。
     */
    void open();

    /**
     * @brief 关闭串口。
     *
     * 重复调用是安全的。关闭失败不会抛出异常，析构函数也使用该函数释放设备。
     */
    void close() noexcept;

    /// 返回串口文件描述符当前是否有效。
    [[nodiscard]] bool isOpen() const noexcept;

    /**
     * @brief 从串口读取一批原始字节。
     *
     * 函数最多等待 SerialConfig::read_timeout_ms。超时且没有数据时返回 0；读取到数据时
     * 返回实际字节数。
     *
     * @param buffer 接收缓冲区首地址；capacity 为 0 时允许为 nullptr。
     * @param capacity 缓冲区可容纳的最大字节数。
     * @throws std::logic_error 串口尚未打开。
     * @throws std::invalid_argument capacity 大于 0 但 buffer 为 nullptr。
     * @throws std::system_error 等待或读取设备失败。
     */
    std::size_t read(std::uint8_t* buffer, std::size_t capacity);

    /**
     * @brief 将指定数据完整写入串口。
     *
     * POSIX write() 可能只写入部分数据，因此该函数会循环，直到全部字节写完或发生错误。
     * 返回时不保证数据已从硬件发送完毕，只保证数据已提交给内核串口缓冲区。
     *
     * @param data 待发送数据的首地址；size 为 0 时允许为 nullptr。
     * @param size 要发送的字节数。
     * @throws std::logic_error 串口尚未打开。
     * @throws std::invalid_argument size 大于 0 但 data 为 nullptr。
     * @throws std::system_error 写入设备失败。
     */
    void write(const std::uint8_t* data, std::size_t size);

private:
    SerialConfig config_;
    int file_descriptor_{-1};
};

} // namespace dart_vision::serial

#endif // DART_SERIAL_SERIAL_PORT_HPP
