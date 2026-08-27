#include "dart_serial/packet.hpp"

#include <cstring>
#include <type_traits>

#include "dart_serial/crc.hpp"

namespace dart_vision::serial {
namespace {

/**
 * @brief 将可按位复制的协议结构体转换为独立字节数组。
 */
template <typename Packet>
std::vector<std::uint8_t> packetToBytes(const Packet& packet) {
    static_assert(
        std::is_trivially_copyable_v<Packet>, "Protocol packet must be trivially copyable");

    std::vector<std::uint8_t> bytes(sizeof(Packet));
    std::memcpy(bytes.data(), &packet, sizeof(Packet));
    return bytes;
}

} // namespace

PacketType packetTypeFromHeader(std::uint8_t header) noexcept {
    switch (header) {
        case kReceivePacketHeader:
            return PacketType::kReceive;
        case kSendPacketHeader:
            return PacketType::kSend;
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
        case PacketType::kUnknown:
            return 0;
    }

    return 0;
}

std::vector<std::uint8_t> encodeSendPacket(SendPacket packet) {
    packet.header = kSendPacketHeader;
    packet.crc = 0;

    auto bytes = packetToBytes(packet);
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

    static_assert(
        std::is_trivially_copyable_v<ReceivePacket>,
        "ReceivePacket must be trivially copyable");

    ReceivePacket packet{};
    std::memcpy(&packet, frame.data(), sizeof(packet));
    return packet;
}

} // namespace dart_vision::serial
