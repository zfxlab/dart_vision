#ifndef DART_SERIAL_PACKET_HPP
#define DART_SERIAL_PACKET_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace dart_vision::serial {

inline constexpr std::uint8_t kReceivePacketHeader = 0x5A;
inline constexpr std::uint8_t kSendPacketHeader = 0xA5;

/**
 * @brief 下位机发送给视觉端的固定长度协议包。
 */
struct ReceivePacket {
    std::uint8_t header{kReceivePacketHeader};
    std::uint8_t target_id{}; ///< 0前哨站 1固定 2随机固定 3随机移动 4末端移动
    float offset_rad{};       ///< 飞镖的偏转角
    float yaw_rad{};          ///< yaw轴电机位置
    std::uint16_t crc{};
} __attribute__((packed));

/**
 * @brief 视觉端发送给下位机的固定长度协议包。
 */
struct SendPacket {
    std::uint8_t header{kSendPacketHeader};
    std::uint8_t state{}; ///< 目标状态
    float yaw_rad{};      ///< 加上offset后的相对偏转,与目标瞄准方向的偏角
    float distance_m{};   ///< 飞镖...与装甲板中心距离(待定)
    std::uint16_t crc{};
} __attribute__((packed));

static_assert(sizeof(ReceivePacket) == 12, "ReceivePacket protocol size mismatch");
static_assert(offsetof(ReceivePacket, target_id) == 1, "ReceivePacket target_id offset mismatch");
static_assert(offsetof(ReceivePacket, offset_rad) == 2, "ReceivePacket offset_rad field mismatch");
static_assert(offsetof(ReceivePacket, yaw_rad) == 6, "ReceivePacket yaw_rad offset mismatch");
static_assert(offsetof(ReceivePacket, crc) == 10, "ReceivePacket CRC offset mismatch");
static_assert(sizeof(SendPacket) == 12, "SendPacket protocol size mismatch");
static_assert(offsetof(SendPacket, state) == 1, "SendPacket state offset mismatch");
static_assert(offsetof(SendPacket, yaw_rad) == 2, "SendPacket yaw_rad offset mismatch");
static_assert(offsetof(SendPacket, distance_m) == 6, "SendPacket distance_m offset mismatch");
static_assert(offsetof(SendPacket, crc) == 10, "SendPacket CRC offset mismatch");

/**
 * @brief 串口协议包类型。
 */
enum class PacketType {
    kReceive, ///< 下位机发送给视觉端的数据包。
    kSend,    ///< 视觉端发送给下位机的数据包。
    kUnknown  ///< 未知帧头，无法判断包类型。
};

/**
 * @brief 根据首字节判断协议包类型。
 *
 * @param header 待判断的帧头。
 * @return 与帧头对应的包类型；无法识别时返回 PacketType::kUnknown。
 */
[[nodiscard]] PacketType packetTypeFromHeader(std::uint8_t header) noexcept;

/**
 * @brief 根据帧头返回对应协议包的完整字节数。
 *
 * @param header 协议帧头。
 * @return ReceivePacket 或 SendPacket 的固定长度；未知帧头返回 0。
 */
[[nodiscard]] std::size_t packetSizeFromHeader(std::uint8_t header) noexcept;

/**
 * @brief 将发送包转换为可写入串口的连续字节，并计算末尾 CRC16。
 *
 * 函数会强制写入 kSendPacketHeader，并覆盖传入包原有的 crc 字段。参数按值传递，因此不会
 * 修改调用方持有的 SendPacket。
 *
 * @param packet 待编码的发送数据。
 * @return 长度固定为 sizeof(SendPacket) 的完整协议帧。
 */
[[nodiscard]] std::vector<std::uint8_t> encodeSendPacket(SendPacket packet);

/**
 * @brief 校验并解码下位机接收帧。
 *
 * 依次检查帧长度、帧头和 CRC16，全部通过后才复制到 ReceivePacket。该函数自身执行 CRC
 * 校验，因此即使调用方没有先经过 PacketParser，也不会接受损坏数据。
 *
 * @param frame 包含帧头、负载和 CRC16 的完整字节帧。
 * @return 校验成功时返回 ReceivePacket，否则返回 std::nullopt。
 */
[[nodiscard]] std::optional<ReceivePacket>
decodeReceivePacket(const std::vector<std::uint8_t>& frame);

} // namespace dart_vision::serial

#endif // DART_SERIAL_PACKET_HPP
