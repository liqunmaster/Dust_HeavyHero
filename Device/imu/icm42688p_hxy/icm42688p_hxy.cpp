#include "icm42688p_hxy.hpp"

namespace {

int16_t read_i16_be(const uint8_t *data)
{
    const uint16_t value = (static_cast<uint16_t>(data[0]) << 8U) | static_cast<uint16_t>(data[1]);
    return static_cast<int16_t>(value);
}

}

int icm42688phxy::decode_frame(const uint8_t *frame, size_t length, icm42688phxy_sample &sample)
{
    if (frame == nullptr || length != frame_size) {
        return -EINVAL;
    }

    icm42688phxy_sample decoded{};
    for (size_t axis = 0U; axis < 3U; ++axis) {
        decoded.accel_raw[axis] = read_i16_be(&frame[axis * 2U]);
        decoded.gyro_raw[axis]  = read_i16_be(&frame[6U + axis * 2U]);
    }
    decoded.temperature_raw = read_i16_be(&frame[ICM42688PHXY_SENSOR_FRAME_SIZE]);

    sample = decoded;
    return 0;
}
