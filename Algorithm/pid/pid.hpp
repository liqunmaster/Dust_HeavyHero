#pragma once

enum class PIDDerivativeMode {
    Error,
    Measurement,
};

struct PIDConfig {
    float kp{};
    
    float ki{};

    float kd{};

    float kf{};

    float dead_zone{};

    float integral_output_limit{};

    float output_limit{};

    float integral_separation_threshold{};

    float variable_integral_start{};

    float variable_integral_end{};

    float derivative_filter_tau{};

    PIDDerivativeMode derivative_mode{PIDDerivativeMode::Measurement};
};

class PIDController {

public:

    void configure(const PIDConfig &config);

    void set_gains(float kp, float ki, float kd, float kf = 0.0F);

    void set_output_limit(float limit);

    void set_integral_output_limit(float limit);

    float update(float target, float measured, float dt);

    float update_angle(float target_rad, float measured_rad, float dt);

    void reset();

    float output() const;

    float integral_output() const;

private:

    float update_error(float target, float measured, float error, float dt, bool angular);

    PIDConfig config_{};

    float integral_output_{};

    float derivative_filtered_{};

    float previous_error_{};

    float previous_measurement_{};

    float output_{};

    bool have_previous_{};
};
