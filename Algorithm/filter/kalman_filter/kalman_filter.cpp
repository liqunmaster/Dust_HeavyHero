#include "kalman_filter.hpp"

#include <math.h>

void KalmanFilter1D::configure(float process_noise, float measurement_noise,
                               float initial_estimate,
                               float initial_covariance)
{
    process_noise_ = process_noise > 0.0F ? process_noise : 0.0F;
    measurement_noise_ = measurement_noise > 0.0F ? measurement_noise : 1.0F;
    reset(initial_estimate, initial_covariance);
}

float KalmanFilter1D::update(float measurement)
{
    if (!isfinite(measurement)) return estimate_;
    covariance_ += process_noise_;
    const float denominator = covariance_ + measurement_noise_;
    if (denominator <= 0.0F || !isfinite(denominator)) return estimate_;
    const float gain = covariance_ / denominator;
    estimate_ += gain * (measurement - estimate_);
    covariance_ *= 1.0F - gain;
    return estimate_;
}

void KalmanFilter1D::reset(float estimate, float covariance)
{
    estimate_ = isfinite(estimate) ? estimate : 0.0F;
    covariance_ = isfinite(covariance) && covariance > 0.0F ? covariance : 1.0F;
}

float KalmanFilter1D::estimate() const { return estimate_; }
float KalmanFilter1D::covariance() const { return covariance_; }

void KalmanAngleBias::configure(float angle_noise, float bias_noise,
                                float measurement_noise)
{
    angle_noise_ = angle_noise > 0.0F ? angle_noise : 0.001F;
    bias_noise_ = bias_noise > 0.0F ? bias_noise : 0.003F;
    measurement_noise_ = measurement_noise > 0.0F ? measurement_noise : 0.03F;
    reset();
}

float KalmanAngleBias::update(float measured_angle, float gyro_rate, float dt,
                             bool measurement_valid)
{
    if (!isfinite(gyro_rate) || !isfinite(dt) || dt <= 0.0F) return angle_;

    angle_ += dt * (gyro_rate - bias_);
    p00_ += dt * (dt * p11_ - p01_ - p10_ + angle_noise_);
    p01_ -= dt * p11_;
    p10_ -= dt * p11_;
    p11_ += bias_noise_ * dt;

    if (!measurement_valid || !isfinite(measured_angle)) return angle_;
    const float innovation = remainderf(
        measured_angle - angle_, 6.28318530717958647692F);

    const float innovation_covariance = p00_ + measurement_noise_;
    if (innovation_covariance <= 0.0F || !isfinite(innovation_covariance)) {
        return angle_;
    }
    const float k0 = p00_ / innovation_covariance;
    const float k1 = p10_ / innovation_covariance;
    angle_ += k0 * innovation;
    bias_ += k1 * innovation;

    const float old_p00 = p00_;
    const float old_p01 = p01_;
    p00_ -= k0 * old_p00;
    p01_ -= k0 * old_p01;
    p10_ -= k1 * old_p00;
    p11_ -= k1 * old_p01;
    return angle_;
}

void KalmanAngleBias::reset(float angle)
{
    angle_ = isfinite(angle) ? angle : 0.0F;
    bias_ = 0.0F;
    p00_ = 1.0F;
    p01_ = 0.0F;
    p10_ = 0.0F;
    p11_ = 1.0F;
}

float KalmanAngleBias::angle() const { return angle_; }
float KalmanAngleBias::bias() const { return bias_; }
