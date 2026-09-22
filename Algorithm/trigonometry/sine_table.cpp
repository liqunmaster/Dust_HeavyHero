#include "sine_table.hpp"

#include <math.h>

static constexpr float kPi = 3.14159265358979323846F;
static constexpr float kTwoPi = 2.0F * kPi;
static constexpr uint16_t kTableSize = 256U;

void SineTable::initialize()
{
    for (uint16_t i = 0U; i < kTableSize; ++i) {
        values_[i] = sinf(kTwoPi * static_cast<float>(i) /
                          static_cast<float>(kTableSize));
    }
}

float SineTable::sine(float angle_rad) const
{
    if (!isfinite(angle_rad)) return 0.0F;
    float wrapped = fmodf(angle_rad, kTwoPi);
    if (wrapped < 0.0F) wrapped += kTwoPi;
    const float position = wrapped *
        (static_cast<float>(kTableSize) / kTwoPi);
    const uint16_t index = static_cast<uint16_t>(position) & 0xFFU;
    const uint16_t next = (index + 1U) & 0xFFU;
    const float fraction = position - static_cast<float>(
        static_cast<uint16_t>(position));
    return values_[index] + fraction * (values_[next] - values_[index]);
}

float SineTable::cosine(float angle_rad) const
{
    return sine(angle_rad + kPi * 0.5F);
}
