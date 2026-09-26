#pragma once

#include <stdint.h>

#ifdef __cplusplus
namespace math {

    constexpr float pi = 3.14159265358979323846F;
    constexpr float two_pi = 2.0F * pi;

    constexpr float encoder_to_turns(int64_t encoder, uint32_t resolution, float gear_ratio = 1.0F)
    {
        return static_cast<float>(encoder) /  (static_cast<float>(resolution) * gear_ratio);
    }

    constexpr float encoder_to_radian(int64_t encoder, uint32_t resolution, float gear_ratio = 1.0F)
    {
        return encoder_to_turns(encoder, resolution, gear_ratio) * two_pi;
    } 

    constexpr float encoder_to_degree(int64_t encoder, uint32_t resolution, float gear_ratio = 1.0F)
    {
        return encoder_to_turns(encoder, resolution, gear_ratio) * 360.0F;
    }

    constexpr float rpm_to_radian_per_second(float rpm, float gear_ratio = 1.0F)
    {
        return rpm * two_pi / (60.0F * gear_ratio);
    }

    constexpr float raw_to_current(int16_t raw, int16_t raw_limit, float current_limit)
    {
        return static_cast<float>(raw) * current_limit / static_cast<float>(raw_limit);
    }

    constexpr int16_t current_to_raw(float current, float current_limit, int16_t raw_limit)
    {
        const float limited = current > current_limit ? current_limit : (current < -current_limit ? -current_limit : current);
        return static_cast<int16_t>(limited * static_cast<float>(raw_limit) / current_limit);
    }

} 
#endif
