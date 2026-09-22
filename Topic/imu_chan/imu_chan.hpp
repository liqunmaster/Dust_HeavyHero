#pragma once

#include <stdint.h>

struct ImuTopicData {
    float quaternion[4];
    float gyro_rad_s[3];
    float accel_m_s2[3];
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float temperature_c;
    bool valid;
    uint32_t timestamp_ms;
};

int imu_topic_publish(const ImuTopicData &data);
int imu_topic_read(ImuTopicData &data);
