#include "attitude_estimator.hpp"

#include <math.h>

void AttitudeEstimator::configure(float angle_noise, float bias_noise,
                                  float measurement_noise)
{
    sine_table_.initialize();
    roll_filter_.configure(angle_noise, bias_noise, measurement_noise);
    pitch_filter_.configure(angle_noise, bias_noise, measurement_noise);
    reset();
}

const AttitudeState &AttitudeEstimator::update(const float accel_m_s2[3],
                                                const float gyro_rad_s[3],
                                                float dt)
{
    if (accel_m_s2 == nullptr || gyro_rad_s == nullptr || !isfinite(dt) ||
        dt <= 0.0F || dt > 0.1F) {
        return state_;
    }
    for (size_t axis = 0U; axis < 3U; ++axis) {
        if (!isfinite(accel_m_s2[axis]) || !isfinite(gyro_rad_s[axis])) {
            return state_;
        }
    }

    const float ax = accel_m_s2[0];
    const float ay = accel_m_s2[1];
    const float az = accel_m_s2[2];
    const float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    const bool accel_valid = isfinite(accel_norm) && accel_norm > 0.8F * 9.80665F &&
                             accel_norm < 1.2F * 9.80665F;
    const float accel_roll = atan2f(ay, az);
    const float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    if (!initialized_ && accel_valid) {
        roll_filter_.reset(accel_roll);
        pitch_filter_.reset(accel_pitch);
        initialized_ = true;
    }
    state_.roll_rad = roll_filter_.update(accel_roll, gyro_rad_s[0], dt,
                                          accel_valid);
    state_.pitch_rad = pitch_filter_.update(accel_pitch, gyro_rad_s[1], dt,
                                            accel_valid);
    state_.yaw_rad += gyro_rad_s[2] * dt;
    state_.orientation = Quaternion::from_euler(state_.roll_rad,
                                                state_.pitch_rad,
                                                state_.yaw_rad,
                                                sine_table_);
    return state_;
}

void AttitudeEstimator::reset()
{
    roll_filter_.reset();
    pitch_filter_.reset();
    state_ = {};
    initialized_ = false;
}

const AttitudeState &AttitudeEstimator::state() const { return state_; }
