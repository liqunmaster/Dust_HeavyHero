#include "pid.hpp"

#include <math.h>

static constexpr float kPi = 3.14159265358979323846F;
static constexpr float kTwoPi = 2.0F * kPi;

static float clamp_symmetric(float value, float limit)
{
    if (limit <= 0.0F) return value;
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float wrap_radians(float angle)
{
    return remainderf(angle, kTwoPi);
}

void PIDController::configure(const PIDConfig &config)
{
    config_ = config;
    reset();
}

void PIDController::set_gains(float kp, float ki, float kd, float kf)
{
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
    config_.kf = kf;
}

void PIDController::set_output_limit(float limit)
{
    config_.output_limit = limit > 0.0F ? limit : 0.0F;
    output_ = clamp_symmetric(output_, config_.output_limit);
}

void PIDController::set_integral_output_limit(float limit)
{
    config_.integral_output_limit = limit > 0.0F ? limit : 0.0F;
    integral_output_ = clamp_symmetric(integral_output_,
                                       config_.integral_output_limit);
}

float PIDController::update(float target, float measured, float dt)
{
    return update_error(target, measured, target - measured, dt, false);
}

float PIDController::update_angle(float target_rad, float measured_rad,
                                  float dt)
{
    if (!isfinite(target_rad) || !isfinite(measured_rad)) {
        output_ = 0.0F;
        return output_;
    }
    return update_error(target_rad, measured_rad,
                        wrap_radians(target_rad - measured_rad), dt, true);
}

float PIDController::update_error(float target, float measured, float raw_error,
                                  float dt, bool angular)
{
    if (!isfinite(target) || !isfinite(measured) || !isfinite(raw_error) ||
        !isfinite(dt) || dt <= 0.0F) {
        output_ = 0.0F;
        return output_;
    }

    const float magnitude = fabsf(raw_error);
    float error = raw_error;
    if (magnitude <= config_.dead_zone) {
        error = 0.0F;
    } else if (config_.dead_zone > 0.0F) {
        error += raw_error > 0.0F ? -config_.dead_zone : config_.dead_zone;
    }

    float derivative = 0.0F;
    if (have_previous_) {
        if (config_.derivative_mode == PIDDerivativeMode::Measurement) {
            float measurement_delta = measured - previous_measurement_;
            if (angular) measurement_delta = wrap_radians(measurement_delta);
            derivative = -measurement_delta / dt;
        } else {
            float error_delta = error - previous_error_;
            if (angular) error_delta = wrap_radians(error_delta);
            derivative = error_delta / dt;
        }
    }

    if (config_.derivative_filter_tau > 0.0F) {
        const float alpha = config_.derivative_filter_tau /
                            (config_.derivative_filter_tau + dt);
        derivative_filtered_ = alpha * derivative_filtered_ +
                               (1.0F - alpha) * derivative;
    } else {
        derivative_filtered_ = derivative;
    }

    float integral_scale = 1.0F;
    const float start = config_.variable_integral_start;
    const float end = config_.variable_integral_end;
    if (end > start && magnitude > start) {
        integral_scale = magnitude >= end ? 0.0F :
            (end - magnitude) / (end - start);
    }
    const bool integrate = config_.integral_separation_threshold <= 0.0F ||
                           magnitude <= config_.integral_separation_threshold;
    float candidate_integral = integral_output_;
    if (integrate) {
        candidate_integral += config_.ki * error * dt * integral_scale;
        candidate_integral = clamp_symmetric(
            candidate_integral, config_.integral_output_limit);
    }

    float candidate_output = config_.kp * error + candidate_integral +
        config_.kd * derivative_filtered_ + config_.kf * target;
    const float limited_output = clamp_symmetric(candidate_output,
                                                  config_.output_limit);
    const bool saturating_further = limited_output != candidate_output &&
        ((candidate_output > 0.0F && error > 0.0F) ||
         (candidate_output < 0.0F && error < 0.0F));
    if (saturating_further && integrate) {
        candidate_integral = integral_output_;
        candidate_output = config_.kp * error + candidate_integral +
            config_.kd * derivative_filtered_ + config_.kf * target;
    } else {
        integral_output_ = candidate_integral;
    }

    output_ = clamp_symmetric(candidate_output, config_.output_limit);
    previous_error_ = error;
    previous_measurement_ = measured;
    have_previous_ = true;
    return output_;
}

void PIDController::reset()
{
    integral_output_ = 0.0F;
    derivative_filtered_ = 0.0F;
    previous_error_ = 0.0F;
    previous_measurement_ = 0.0F;
    output_ = 0.0F;
    have_previous_ = false;
}

float PIDController::output() const
{
    return output_;
}

float PIDController::integral_output() const
{
    return integral_output_;
}
