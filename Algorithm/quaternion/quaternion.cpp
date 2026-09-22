#include "quaternion.hpp"

#include "sine_table.hpp"

#include <math.h>

void Quaternion::normalize()
{
    const float norm = sqrtf(w * w + x * x + y * y + z * z);
    if (!isfinite(norm) || norm < 1.0e-8F) {
        w = 1.0F;
        x = y = z = 0.0F;
        return;
    }
    const float inverse = 1.0F / norm;
    w *= inverse;
    x *= inverse;
    y *= inverse;
    z *= inverse;
}

void Quaternion::integrate_gyro(const float gyro_rad_s[3], float dt)
{
    if (gyro_rad_s == nullptr || !isfinite(dt) || dt <= 0.0F) return;
    const float gx = gyro_rad_s[0];
    const float gy = gyro_rad_s[1];
    const float gz = gyro_rad_s[2];
    const float half_dt = 0.5F * dt;
    const float dw = -x * gx - y * gy - z * gz;
    const float dx =  w * gx + y * gz - z * gy;
    const float dy =  w * gy - x * gz + z * gx;
    const float dz =  w * gz + x * gy - y * gx;
    w += half_dt * dw;
    x += half_dt * dx;
    y += half_dt * dy;
    z += half_dt * dz;
    normalize();
}

Quaternion Quaternion::from_euler(float roll_rad, float pitch_rad,
                                   float yaw_rad)
{
    const float cr = cosf(roll_rad * 0.5F);
    const float sr = sinf(roll_rad * 0.5F);
    const float cp = cosf(pitch_rad * 0.5F);
    const float sp = sinf(pitch_rad * 0.5F);
    const float cy = cosf(yaw_rad * 0.5F);
    const float sy = sinf(yaw_rad * 0.5F);
    Quaternion q{};
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.normalize();
    return q;
}

Quaternion Quaternion::from_euler(float roll_rad, float pitch_rad,
                                   float yaw_rad, const SineTable &table)
{
    const float cr = table.cosine(roll_rad * 0.5F);
    const float sr = table.sine(roll_rad * 0.5F);
    const float cp = table.cosine(pitch_rad * 0.5F);
    const float sp = table.sine(pitch_rad * 0.5F);
    const float cy = table.cosine(yaw_rad * 0.5F);
    const float sy = table.sine(yaw_rad * 0.5F);
    Quaternion q{};
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.normalize();
    return q;
}

void Quaternion::to_euler(float *roll_rad, float *pitch_rad,
                          float *yaw_rad) const
{
    if (roll_rad != nullptr) {
        *roll_rad = atan2f(2.0F * (w * x + y * z),
                           1.0F - 2.0F * (x * x + y * y));
    }
    if (pitch_rad != nullptr) {
        float value = 2.0F * (w * y - z * x);
        if (value > 1.0F) value = 1.0F;
        if (value < -1.0F) value = -1.0F;
        *pitch_rad = asinf(value);
    }
    if (yaw_rad != nullptr) {
        *yaw_rad = atan2f(2.0F * (w * z + x * y),
                          1.0F - 2.0F * (y * y + z * z));
    }
}
