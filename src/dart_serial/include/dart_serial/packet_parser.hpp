#ifndef DART_SERIAL_PACKET_PARSER_HPP
#define DART_SERIAL_PACKET_PARSER_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dart_vision::serial {

/**
 * @brief 一次解析操作的状态。
 *
 * PacketParser 面向连续字节流工作。一次 nextFrame() 调用最多报告一个事件，调用方应在收到
 * kFrameReady、kCRCError 或 kUnknownHeader 后继续调用，直到返回 kNeedMoreData。
 */
enum class ParseStatus {
    kNeedMoreData, ///< 缓冲区为空，或只有一段尚未接收完整的合法帧。
    kFrameReady,   ///< 已取得一帧长度正确且 CRC16 校验通过的数据。
    kCRCError,     ///< 找到完整候选帧，但其 CRC16 校验失败。
    kUnknownHeader ///< 丢弃了合法接收帧头之前的无效字节。
};

/**
 * @brief nextFrame() 返回的解析结果。
 */
struct ParseResult {
    ParseStatus status{ParseStatus::kNeedMoreData}; ///< 本次解析状态。

    /**
     * @brief 与本次状态关联的字节。
     *
     * kFrameReady 时保存校验通过的完整帧；kCRCError 时保存 CRC 错误的完整候选帧；
     * kUnknownHeader 时保存本次丢弃的无效前导字节；kNeedMoreData 时为空。
     */
    std::vector<std::uint8_t> frame;
};

/**
 * @brief 从串口连续字节流中提取固定长度协议帧。
 *
 * 串口的一次 read() 不保证对应一帧：可能只读到半帧，也可能一次读到多帧。PacketParser
 * 通过内部缓冲区跨 read() 保存数据，并根据 packet.hpp 中定义的接收帧头和结构体大小分帧。
 *
 * 当前解析器只接收 ReceivePacket。SendPacket 是本机发往下位机的数据，因此其帧头不会被
 * 当作合法接收帧头。
 *
 * @note 该类本身不提供线程安全保证，应由同一个接收线程调用，或由调用方进行同步。
 */
class PacketParser {
public:
    /**
     * @brief 将新读取的串口字节追加到内部缓冲区。
     *
     * @param data 新数据的首地址；size 为 0 时允许为 nullptr。
     * @param size 要追加的字节数。
     * @throws std::invalid_argument size 大于 0 但 data 为 nullptr。
     */
    void append(const std::uint8_t* data, std::size_t size);

    /**
     * @brief 尝试从缓冲区解析一个事件或一帧数据。
     *
     * 处理顺序为：寻找合法接收帧头、丢弃前导噪声、等待固定长度数据、验证 CRC16，最后
     * 返回完整帧。CRC 错误时仅移除候选帧的第一个字节，使下一次调用能重新搜索帧头并恢复
     * 同步，避免错误候选帧覆盖其内部可能存在的下一帧帧头。
     *
     * @return 本次解析结果；调用方应持续调用，直到状态为 kNeedMoreData。
     */
    [[nodiscard]] ParseResult nextFrame();

    /**
     * @brief 清空所有尚未解析的缓存数据。
     *
     * 通常在串口断开、重新打开或通信协议状态需要重新同步时调用。
     */
    void reset();

    /**
     * @brief 返回当前尚未消费的缓存字节数。
     */
    [[nodiscard]] std::size_t bufferedSize() const noexcept;

private:
    std::vector<std::uint8_t> buffer_;
};

} // namespace dart_vision::serial

#endif // DART_VISION_SERIAL_PACKET_PARSER_HPP
