#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include "icm42688p_hxy_reg.h"

struct icm42688phxy_sample {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temperature_raw;
};

class icm42688phxy final {
public:
    static constexpr size_t frame_size = ICM42688PHXY_SENSOR_FRAME_SIZE + ICM42688PHXY_TEMP_FRAME_SIZE;

    static int decode_frame(const uint8_t *frame, size_t length, icm42688phxy_sample &sample);
};
