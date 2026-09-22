#pragma once

class KalmanFilter1D {
public:
    void configure(float process_noise, float measurement_noise,
                   float initial_estimate = 0.0F,
                   float initial_covariance = 1.0F);
    float update(float measurement);
    void reset(float estimate = 0.0F, float covariance = 1.0F);
    float estimate() const;
    float covariance() const;

private:
    float process_noise_{};
    float measurement_noise_{1.0F};
    float estimate_{};
    float covariance_{1.0F};
};

class KalmanAngleBias {
public:
    void configure(float angle_noise, float bias_noise,
                   float measurement_noise);
    float update(float measured_angle, float gyro_rate, float dt,
                 bool measurement_valid = true);
    void reset(float angle = 0.0F);
    float angle() const;
    float bias() const;

private:
    float angle_noise_{0.001F};
    float bias_noise_{0.003F};
    float measurement_noise_{0.03F};
    float angle_{};
    float bias_{};
    float p00_{1.0F};
    float p01_{};
    float p10_{};
    float p11_{1.0F};
};
