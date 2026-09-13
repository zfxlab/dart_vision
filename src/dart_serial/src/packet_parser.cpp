#include "dart_serial/packet_parser.hpp"

#include <algorithm>
#include <stdexcept>

#include "dart_serial/crc.hpp"
#include "dart_serial/packet.hpp"

namespace dart_vision::serial {
namespace {

bool isIncomingHeader(std::uint8_t value) noexcept {
    return value == kReceivePacketHeader || value == kLoggerPacketHeader;
}

} // namespace

void PacketParser::append(const std::uint8_t* data, std::size_t size) {
    if (size == 0) {
        return;
    }
    if (data == nullptr) {
        throw std::invalid_argument(
            "PacketParser::append data must not be null when size is non-zero");
    }

    buffer_.insert(buffer_.end(), data, data + size);
}

ParseResult PacketParser::nextFrame() {
    if (buffer_.empty()) {
        return {};
    }

    const auto header = std::find_if(buffer_.cbegin(), buffer_.cend(), isIncomingHeader);

    if (header != buffer_.cbegin()) {
        ParseResult result;
        result.status = ParseStatus::kUnknownHeader;
        result.frame.assign(buffer_.cbegin(), header);
        buffer_.erase(buffer_.begin(), buffer_.begin() + result.frame.size());
        return result;
    }

    const std::size_t frame_size = packetSizeFromHeader(buffer_.front());
    if (buffer_.size() < frame_size) {
        return {};
    }

    ParseResult result;
    result.frame.assign(buffer_.cbegin(), buffer_.cbegin() + frame_size);

    if (!verifyCRC16(result.frame.data(), result.frame.size())) {
        result.status = ParseStatus::kCRCError;
        buffer_.erase(buffer_.begin());
        return result;
    }

    result.status = ParseStatus::kFrameReady;
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
    return result;
}

void PacketParser::reset() {
    buffer_.clear();
}

std::size_t PacketParser::bufferedSize() const noexcept {
    return buffer_.size();
}

} // namespace dart_vision::serial
