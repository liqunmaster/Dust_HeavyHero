#include "vt03.hpp"

#include <errno.h>

#include "crc.hpp"

namespace {
int16_t signed_word(const uint8_t *bytes)
{
    const int32_t value = static_cast<int32_t>(bytes[0]) |
        (static_cast<int32_t>(bytes[1]) << 8U);
    return static_cast<int16_t>(value < 0x8000 ? value : value - 0x10000);
}
}

int vt03::decode_frame(const uint8_t *frame, size_t length, vt03_sample &sample)
{
    if (frame == nullptr || length != frame_size) return -EINVAL;
    if (frame[0] != 0xA9U || frame[1] != 0x53U) return -EBADMSG;

    const uint16_t received_crc = static_cast<uint16_t>(frame[19]) |
        (static_cast<uint16_t>(frame[20]) << 8U);
    if (crc16_ccitt_false(frame, 19U) != received_crc) return -EBADMSG;

    uint64_t controls = 0U;
    for (size_t i = 0U; i < 8U; ++i) {
        controls |= static_cast<uint64_t>(frame[2U + i]) << (8U * i);
    }

    vt03_sample decoded{};
    for (size_t i = 0U; i < 4U; ++i) {
        decoded.channel[i] = static_cast<uint16_t>((controls >> (11U * i)) & 0x07FFU);
        if (decoded.channel[i] < 364U || decoded.channel[i] > 1684U) return -EBADMSG;
    }
    decoded.mode_switch = static_cast<uint8_t>((controls >> 44U) & 0x03U);
    decoded.pause = ((controls >> 46U) & 0x01U) != 0U;
    decoded.custom_left = ((controls >> 47U) & 0x01U) != 0U;
    decoded.custom_right = ((controls >> 48U) & 0x01U) != 0U;
    decoded.wheel = static_cast<uint16_t>((controls >> 49U) & 0x07FFU);
    decoded.trigger = ((controls >> 60U) & 0x01U) != 0U;
    if (decoded.mode_switch > 2U || decoded.wheel < 364U || decoded.wheel > 1684U) return -EBADMSG;

    decoded.mouse_x = signed_word(frame + 10U);
    decoded.mouse_y = signed_word(frame + 12U);
    decoded.mouse_z = signed_word(frame + 14U);
    if ((frame[16] & 0x03U) > 1U ||
        ((frame[16] >> 2U) & 0x03U) > 1U ||
        ((frame[16] >> 4U) & 0x03U) > 1U) return -EBADMSG;
    decoded.mouse_left = (frame[16] & 0x03U) != 0U;
    decoded.mouse_right = ((frame[16] >> 2U) & 0x03U) != 0U;
    decoded.mouse_middle = ((frame[16] >> 4U) & 0x03U) != 0U;
    decoded.keyboard = static_cast<uint16_t>(frame[17]) |
        (static_cast<uint16_t>(frame[18]) << 8U);

    sample = decoded;
    return 0;
}
