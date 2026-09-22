#pragma once

#include "kalman_filter.hpp"
#include "quaternion.hpp"
#include "sine_table.hpp"

struct AttitudeState {
    Quaternion orientation{};
    float roll_rad{};
    float pitch_rad{};
    float yaw_rad{};
};

class AttitudeEstimator {
public:
    void configure(float angle_noise, float bias_noise,
                   float measurement_noise);
    const AttitudeState &update(const float accel_m_s2[3],
                                const float gyro_rad_s[3], float dt);
    void reset();
    const AttitudeState &state() const;

private:
    KalmanAngleBias roll_filter_{};
    KalmanAngleBias pitch_filter_{};
    AttitudeState state_{};
    SineTable sine_table_{};
    bool initialized_{};
};
