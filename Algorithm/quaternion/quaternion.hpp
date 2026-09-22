#pragma once

class SineTable;

struct Quaternion {
    float w{1.0F};
    float x{};
    float y{};
    float z{};

    void normalize();
    void integrate_gyro(const float gyro_rad_s[3], float dt);
    static Quaternion from_euler(float roll_rad, float pitch_rad,
                                 float yaw_rad);
    static Quaternion from_euler(float roll_rad, float pitch_rad,
                                 float yaw_rad, const SineTable &table);
    void to_euler(float *roll_rad, float *pitch_rad, float *yaw_rad) const;
};
