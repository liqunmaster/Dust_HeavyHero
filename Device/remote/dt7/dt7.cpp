#include "dt7.hpp"

int dt7::decode_frame(const uint8_t *frame, size_t length, dt7_sample &sample)
{
    if (frame == nullptr || length != frame_size) {
        return -EINVAL;
    }

    uint64_t controls = 0U;
    for (size_t i = 0U; i < 6U; ++i) {
        controls |= static_cast<uint64_t>(frame[i]) << (8U * i);
    }

    dt7_sample decoded{};
    for (size_t i = 0U; i < 4U; ++i) {
        decoded.channel[i] = static_cast<uint16_t>((controls >> (11U * i)) & 0x07FFU);
        if (decoded.channel[i] < 364U || decoded.channel[i] > 1684U) {
            return -EBADMSG;
        }
    }
    decoded.switch_left = static_cast<uint8_t>((controls >> 44U) & 0x03U);
    decoded.switch_right = static_cast<uint8_t>((controls >> 46U) & 0x03U);
    if (decoded.switch_left < 1U || decoded.switch_left > 3U || decoded.switch_right < 1U || decoded.switch_right > 3U) {
        return -EBADMSG;   
    }

    sample = decoded;
    return 0;
}
