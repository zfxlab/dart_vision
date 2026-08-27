#include "dart_serial/serial_port.hpp"

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <unistd.h>

namespace dart_vision::serial {
namespace {

speed_t baudRateToTermios(std::uint32_t baud_rate) {
    switch (baud_rate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
#ifdef B230400
        case 230400:
            return B230400;
#endif
#ifdef B460800
        case 460800:
            return B460800;
#endif
#ifdef B921600
        case 921600:
            return B921600;
#endif
        default:
            throw std::invalid_argument("Unsupported serial baud rate");
    }
}

[[noreturn]] void throwSystemError(const char* operation) {
    throw std::system_error(errno, std::generic_category(), operation);
}

} // namespace

SerialPort::SerialPort(const SerialConfig& config) : config_(config) {
    if (config_.device.empty()) {
        throw std::invalid_argument("Serial device path must not be empty");
    }
    if (config_.read_timeout_ms < 0) {
        throw std::invalid_argument("Serial read timeout must not be negative");
    }

    // 在构造阶段验证波特率，使无效配置尽早暴露，而不是推迟到 open()。
    static_cast<void>(baudRateToTermios(config_.baud_rate));
}

SerialPort::~SerialPort() noexcept {
    close();
}

void SerialPort::open() {
    if (isOpen()) {
        return;
    }

    const int descriptor = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (descriptor == -1) {
        throwSystemError("Failed to open serial device");
    }

    termios options{};
    if (::tcgetattr(descriptor, &options) == -1) {
        const int saved_errno = errno;
        ::close(descriptor);
        errno = saved_errno;
        throwSystemError("Failed to read serial attributes");
    }

    ::cfmakeraw(&options);

    const speed_t speed = baudRateToTermios(config_.baud_rate);
    if (::cfsetispeed(&options, speed) == -1 || ::cfsetospeed(&options, speed) == -1) {
        const int saved_errno = errno;
        ::close(descriptor);
        errno = saved_errno;
        throwSystemError("Failed to set serial baud rate");
    }

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8 | CLOCAL | CREAD;
    options.c_cflag &= ~(PARENB | CSTOPB);
#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 实际等待由 poll() 控制，底层 read() 在 poll 报告可读后立即返回。
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (::tcsetattr(descriptor, TCSANOW, &options) == -1) {
        const int saved_errno = errno;
        ::close(descriptor);
        errno = saved_errno;
        throwSystemError("Failed to configure serial device");
    }

    if (::tcflush(descriptor, TCIOFLUSH) == -1) {
        const int saved_errno = errno;
        ::close(descriptor);
        errno = saved_errno;
        throwSystemError("Failed to flush serial device");
    }

    file_descriptor_ = descriptor;
}

void SerialPort::close() noexcept {
    if (!isOpen()) {
        return;
    }

    const int descriptor = file_descriptor_;
    file_descriptor_ = -1;

    // Linux 上 close() 即使被信号中断，文件描述符也可能已经释放；不能贸然重试，避免
    // descriptor 被其他线程复用后误关新资源。这里在析构路径中忽略关闭结果。
    static_cast<void>(::close(descriptor));
}

bool SerialPort::isOpen() const noexcept {
    return file_descriptor_ >= 0;
}

std::size_t SerialPort::read(std::uint8_t* buffer, std::size_t capacity) {
    if (!isOpen()) {
        throw std::logic_error("Cannot read from a closed serial port");
    }
    if (capacity == 0) {
        return 0;
    }
    if (buffer == nullptr) {
        throw std::invalid_argument("Serial read buffer must not be null");
    }

    pollfd descriptor{};
    descriptor.fd = file_descriptor_;
    descriptor.events = POLLIN;

    int poll_result;
    do {
        poll_result = ::poll(&descriptor, 1, config_.read_timeout_ms);
    } while (poll_result == -1 && errno == EINTR);

    if (poll_result == -1) {
        throwSystemError("Failed while waiting for serial data");
    }
    if (poll_result == 0) {
        return 0;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        throw std::system_error(std::make_error_code(std::errc::io_error),
                                "Serial device became unavailable");
    }

    ssize_t bytes_read;
    do {
        bytes_read = ::read(file_descriptor_, buffer, capacity);
    } while (bytes_read == -1 && errno == EINTR);

    if (bytes_read == -1) {
        throwSystemError("Failed to read from serial device");
    }

    return static_cast<std::size_t>(bytes_read);
}

void SerialPort::write(const std::uint8_t* data, std::size_t size) {
    if (!isOpen()) {
        throw std::logic_error("Cannot write to a closed serial port");
    }
    if (size == 0) {
        return;
    }
    if (data == nullptr) {
        throw std::invalid_argument("Serial write data must not be null");
    }

    std::size_t total_written = 0;
    while (total_written < size) {
        const ssize_t bytes_written =
            ::write(file_descriptor_, data + total_written, size - total_written);

        if (bytes_written > 0) {
            total_written += static_cast<std::size_t>(bytes_written);
            continue;
        }
        if (bytes_written == -1 && errno == EINTR) {
            continue;
        }
        if (bytes_written == 0) {
            throw std::system_error(std::make_error_code(std::errc::io_error),
                                    "Serial write made no progress");
        }

        throwSystemError("Failed to write to serial device");
    }
}

} // namespace dart_vision::serial
