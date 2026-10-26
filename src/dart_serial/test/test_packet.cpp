#include "dart_serial/crc.hpp"
#include "dart_serial/packet.hpp"
#include <algorithm>
#include <gtest/gtest.h>
#include <limits>

using namespace dart_vision::serial;

TEST(SendPacketProtocol, EncodesExplicitLittleEndianGeometryAndCrc) {
    SendPacket packet;
    packet.state = 1;
    packet.yaw_rad = 0.5F;
    packet.horizontal_distance_m = 10.0F;
    packet.vertical_difference_m = -2.0F;
    const auto bytes = encodeSendPacket(packet);
    ASSERT_EQ(bytes.size(), 16U);
    const std::vector<std::uint8_t> expected{0xA5, 1,    0,    0, 0, 0x3F, 0,
                                             0,    0x20, 0x41, 0, 0, 0,    0xC0};
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), bytes.begin()));
    // Independent bit-wise CRC reference: reflected polynomial 0x8408, init 0xffff.
    std::uint16_t crc = 0xffff;
    for (auto byte : expected) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i)
            crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
    }
    EXPECT_EQ(bytes[14], crc & 0xff);
    EXPECT_EQ(bytes[15], crc >> 8);
    EXPECT_TRUE(verifyCRC16(bytes.data(), bytes.size()));
    EXPECT_EQ(packetSizeFromHeader(0xA5), 16U);
    EXPECT_EQ(packetTypeFromHeader(0xA5), PacketType::kSend);
}

TEST(SendPacketProtocol, ClearsGeometryForClosedInvalidAndMalformedCommands) {
    for (std::uint8_t state : {0, 2, 255}) {
        SendPacket packet;
        packet.state = state;
        packet.yaw_rad = 1;
        packet.horizontal_distance_m = 10;
        packet.vertical_difference_m = -2;
        const auto bytes = encodeSendPacket(packet);
        EXPECT_EQ(bytes[1], state == 0 ? 0 : 2);
        EXPECT_TRUE(
            std::all_of(bytes.begin() + 2, bytes.begin() + 14, [](auto b) { return b == 0; }));
    }
    SendPacket packet;
    packet.state = 1;
    packet.horizontal_distance_m = 10;
    for (float height : {-2.0F, 0.0F, 2.0F}) {
        packet.vertical_difference_m = height;
        EXPECT_EQ(encodeSendPacket(packet)[1], 1);
    }
    packet.vertical_difference_m = std::numeric_limits<float>::infinity();
    EXPECT_EQ(encodeSendPacket(packet)[1], 2);
    packet.vertical_difference_m = 0;
    packet.horizontal_distance_m = -1;
    EXPECT_EQ(encodeSendPacket(packet)[1], 2);
}
