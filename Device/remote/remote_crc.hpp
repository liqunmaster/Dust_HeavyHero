#pragma once
#include <stddef.h>
#include <stdint.h>

inline bool remote_crc16_valid(const uint8_t *data, size_t length)
{
    if (data == nullptr || length < 2U) return false;
    uint16_t crc = 0xffffU;
    for (size_t i = 0; i < length - 2U; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit)
            crc = (crc & 1U) ? uint16_t((crc >> 1) ^ 0x8408U)
                             : uint16_t(crc >> 1);
    }
    const uint16_t expected = uint16_t(data[length - 2U]) |
                              (uint16_t(data[length - 1U]) << 8);
    return crc == expected;
}
