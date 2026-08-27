#ifndef DART_SERIAL_CRC_HPP
#define DART_SERIAL_CRC_HPP

#include <cstddef>
#include <cstdint>

namespace dart_vision::serial {

/**
 * @brief 计算指定数据的 CRC16 校验值。
 *
 * @param data 待计算数据的首地址。
 * @param length 数据长度，单位为字节。
 * @return 计算得到的 16 位 CRC 值。
 */
std::uint16_t calculateCRC16(const std::uint8_t* data, std::size_t length);

/**
 * @brief 检验数据的 CRC16 值是否正确。
 *
 * @param data 待检验数据的首地址，不包含 CRC 字段。
 * @param length 数据长度，单位为字节，不包含 CRC 字段。
 * @return true 表示 CRC 正确，false 表示 CRC 错误。
 */
bool verifyCRC16(const std::uint8_t* data, std::size_t length);

/**
 * @brief 计算 CRC16 并将结果添加到数据末尾。
 *
 * @param data 待添加 CRC16 的数据。
 * @param length 数据长度，单位为字节，包含 CRC 字段的两字节。
 */
void appendCRC16(std::uint8_t* data, std::size_t length);

} // namespace dart_vision::serial

#endif // DART_SERIAL_CRC_HPP
