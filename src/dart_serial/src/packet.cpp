#include "dart_serial/packet.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

#include "dart_serial/crc.hpp"

namespace dart_vision::serial {
PacketType packetTypeFromHeader(std::uint8_t header) noexcept {
    switch (header) {
    case kReceivePacketHeader:
        return PacketType::kReceive;
    case kSendPacketHeader:
        return PacketType::kSend;
    case kLoggerPacketHeader:
        return PacketType::kLogger;
    default:
        return PacketType::kUnknown;
    }
}

std::size_t packetSizeFromHeader(std::uint8_t header) noexcept {
    switch (packetTypeFromHeader(header)) {
    case PacketType::kReceive:
        return sizeof(ReceivePacket);
    case PacketType::kSend:
        return sizeof(SendPacket);
    case PacketType::kLogger:
        return sizeof(LoggerPacket);
    case PacketType::kUnknown:
        return 0;
    }

    return 0;
}

std::vector<std::uint8_t> encodeSendPacket(SendPacket packet) {
    packet.header = kSendPacketHeader;
    // CLOSED/INVALID 不携带几何量。任何非法 VALID 数值都降级为 INVALID。
    if (packet.state > 2 ||
        (packet.state == 1 &&
         (!std::isfinite(packet.yaw_rad) || !std::isfinite(packet.horizontal_distance_m) ||
          packet.horizontal_distance_m <= 0.0F || !std::isfinite(packet.vertical_difference_m))))
        packet.state = 2;
    if (packet.state != 1) {
        packet.yaw_rad = 0.0F;
        packet.horizontal_distance_m = 0.0F;
        packet.vertical_difference_m = 0.0F;
    }
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    std::vector<std::uint8_t> bytes(sizeof(SendPacket), 0);
    bytes[0] = packet.header;
    bytes[1] = packet.state;
    const auto write_float = [&bytes](std::size_t offset, float value) {
        std::uint32_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        for (std::size_t index = 0; index < sizeof(bits); ++index)
            bytes[offset + index] = static_cast<std::uint8_t>(bits >> (8U * index));
    };
    write_float(2, packet.yaw_rad);
    write_float(6, packet.horizontal_distance_m);
    write_float(10, packet.vertical_difference_m);
    appendCRC16(bytes.data(), bytes.size());
    return bytes;
}

std::optional<ReceivePacket> decodeReceivePacket(const std::vector<std::uint8_t>& frame) {
    if (frame.size() != sizeof(ReceivePacket)) {
        return std::nullopt;
    }
    if (frame.front() != kReceivePacketHeader) {
        return std::nullopt;
    }
    if (!verifyCRC16(frame.data(), frame.size())) {
        return std::nullopt;
    }

    static_assert(std::is_trivially_copyable_v<ReceivePacket>,
                  "ReceivePacket must be trivially copyable");

    ReceivePacket packet{};
    std::memcpy(&packet, frame.data(), sizeof(packet));
    return packet;
}

std::optional<LoggerPacket> decodeLoggerPacket(const std::vector<std::uint8_t>& frame) {
    if (frame.size() != sizeof(LoggerPacket)) {
        return std::nullopt;
    }
    if (frame.front() != kLoggerPacketHeader) {
        return std::nullopt;
    }
    if (!verifyCRC16(frame.data(), frame.size())) {
        return std::nullopt;
    }

    static_assert(std::is_trivially_copyable_v<LoggerPacket>,
                  "LoggerPacket must be trivially copyable");

    LoggerPacket packet{};
    std::memcpy(&packet, frame.data(), sizeof(packet));
    return packet;
}

} // namespace dart_vision::serial
